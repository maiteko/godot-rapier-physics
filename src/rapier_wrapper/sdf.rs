//! Colliding with a signed distance field: a world whose surface is the zero set of a
//! distance function (a voxel terrain), with no triangles.
//!
//! Another extension linked with this one (the `embedded` feature) implements
//! [`DistanceField`], registers it with [`register_field`], and passes the key as the data
//! of a Godot custom shape (`PhysicsServer3D.custom_shape_create`). The shape is an
//! [`SdfShape`]; [`SdfDispatcher`](super::sdf_dispatcher::SdfDispatcher), first in the
//! narrow phase's dispatcher chain, answers every query between it and other shapes:
//! contacts and manifolds, intersection, distance, linear and nonlinear shape casts.
//! Rays and point queries go through [`SdfShape`]'s own `RayCast` and `PointQuery`.
//!
//! The field must be (about) 1-Lipschitz, as a distance is: shape casts and rays advance
//! by the distance, trusting that nothing is nearer.

use std::collections::HashMap;
use std::sync::Arc;
use std::sync::LazyLock;
use std::sync::Mutex;
use std::sync::atomic::AtomicI64;
use std::sync::atomic::Ordering;

use rapier::parry::bounding_volume::BoundingSphere;
use rapier::parry::mass_properties::MassProperties;
use rapier::parry::query::PointProjection;
use rapier::parry::query::PointQuery;
use rapier::parry::query::Ray;
use rapier::parry::query::RayCast;
use rapier::parry::query::RayIntersection;
use rapier::parry::shape::Ball;
use rapier::parry::shape::Capsule;
use rapier::parry::shape::Compound;
use rapier::parry::shape::FeatureId;
use rapier::parry::shape::PolygonalFeature;
use rapier::parry::shape::Shape;
use rapier::parry::shape::TriMesh;
use rapier::parry::shape::TypedShape;
use rapier::prelude::*;

/// The math types of [`DistanceField`] (glam's, at the build's precision).
pub use rapier::math::{Real, Vector};
pub use rapier::parry::bounding_volume::Aabb;

/// A signed distance field: negative inside matter, positive outside, in the shape's
/// local space and units.
pub trait DistanceField: Send + Sync {
    /// The signed distance from `p` to the surface.
    fn distance(&self, p: Vector) -> Real;

    /// The unit direction away from matter at `p` (the distance's gradient), or zero where
    /// it has none. By default from central differences.
    fn gradient(&self, p: Vector) -> Vector {
        let h = GRADIENT_STEP;
        let axis = |a: Vector| self.distance(p + a * h) - self.distance(p - a * h);
        Vector::new(axis(Vector::X), axis(Vector::Y), axis(Vector::Z)).normalize_or_zero()
    }

    /// Both [`distance`](Self::distance) and [`gradient`](Self::gradient) at `p`: what a
    /// probe needs. Override it when both come cheaper together.
    fn sample(&self, p: Vector) -> (Real, Vector) {
        (self.distance(p), self.gradient(p))
    }

    /// The box (local space) outside which there is no matter. By default very large.
    fn bounds(&self) -> Aabb {
        Aabb::new(Vector::splat(-FAR), Vector::splat(FAR))
    }
}

/// The step of [`DistanceField::gradient`]'s central differences.
pub const GRADIENT_STEP: Real = 0.02;
/// The default bounds' half-size.
const FAR: Real = 1.0e6;

static FIELDS: LazyLock<Mutex<HashMap<i64, Arc<dyn DistanceField>>>> =
    LazyLock::new(|| Mutex::new(HashMap::new()));
static NEXT_KEY: AtomicI64 = AtomicI64::new(1);

/// Register `field` for SDF shapes: returns the key to pass as a custom shape's data.
pub fn register_field(field: Arc<dyn DistanceField>) -> i64 {
    let key = NEXT_KEY.fetch_add(1, Ordering::Relaxed);
    FIELDS.lock().unwrap().insert(key, field);
    key
}

/// Forget the field registered as `key` (shapes made from it keep it until they go).
pub fn unregister_field(key: i64) {
    FIELDS.lock().unwrap().remove(&key);
}

/// The field registered as `key`.
pub fn field(key: i64) -> Option<Arc<dyn DistanceField>> {
    FIELDS.lock().unwrap().get(&key).cloned()
}

/// A shape whose matter is where a [`DistanceField`] is negative.
#[derive(Clone)]
pub struct SdfShape {
    pub key: i64,
    pub field: Arc<dyn DistanceField>,
}

impl SdfShape {
    pub fn new(key: i64, field: Arc<dyn DistanceField>) -> Self {
        Self { key, field }
    }
}

impl std::fmt::Debug for SdfShape {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.debug_struct("SdfShape").field("key", &self.key).finish()
    }
}

impl Shape for SdfShape {
    fn compute_local_aabb(&self) -> Aabb {
        self.field.bounds()
    }

    fn compute_local_bounding_sphere(&self) -> BoundingSphere {
        let aabb = self.field.bounds();
        BoundingSphere::new(aabb.center(), aabb.half_extents().length())
    }

    fn clone_dyn(&self) -> Box<dyn Shape> {
        Box::new(self.clone())
    }

    /// Only unscaled.
    fn scale_dyn(&self, scale: Vector, _num_subdivisions: u32) -> Option<Box<dyn Shape>> {
        (scale == Vector::ONE).then(|| self.clone_dyn())
    }

    /// For static bodies only.
    fn mass_properties(&self, _density: Real) -> MassProperties {
        MassProperties::default()
    }

    fn shape_type(&self) -> ShapeType {
        ShapeType::Custom
    }

    fn as_typed_shape(&self) -> TypedShape<'_> {
        TypedShape::Custom(self)
    }

    fn ccd_thickness(&self) -> Real {
        Real::MAX
    }

    fn ccd_angular_thickness(&self) -> Real {
        std::f64::consts::PI as Real
    }
}

/// Sphere tracing: most steps along a ray.
const MAX_STEPS: usize = 512;
/// Nearer than this, a ray has hit.
const HIT_DISTANCE: Real = 1.0e-3;

impl RayCast for SdfShape {
    fn cast_local_ray_and_get_normal(
        &self,
        ray: &Ray,
        max_time_of_impact: Real,
        solid: bool,
    ) -> Option<RayIntersection> {
        let len = ray.dir.length();
        if len <= 0.0 {
            return None;
        }
        let field = &self.field;
        let start = field.distance(ray.origin);
        if start < 0.0 && solid {
            return Some(RayIntersection::new(0.0, Vector::ZERO, FeatureId::Unknown));
        }
        // From inside (not solid): on to where the ray leaves matter.
        let inside = start < 0.0;
        let mut t = 0.0;
        for _ in 0..MAX_STEPS {
            let p = ray.point_at(t);
            let d = field.distance(p);
            let gap = if inside { -d } else { d };
            if gap < HIT_DISTANCE {
                let (_, normal) = field.sample(p);
                return Some(RayIntersection::new(t, normal, FeatureId::Unknown));
            }
            t += gap.max(HIT_DISTANCE) / len;
            if t > max_time_of_impact {
                return None;
            }
        }
        None
    }
}

impl PointQuery for SdfShape {
    fn project_local_point(&self, pt: Vector, solid: bool) -> PointProjection {
        let (d, normal) = self.field.sample(pt);
        let inside = d < 0.0;
        if inside && solid {
            return PointProjection::new(true, pt);
        }
        PointProjection::new(inside, pt - normal * d)
    }

    fn project_local_point_and_get_feature(&self, pt: Vector) -> (PointProjection, FeatureId) {
        (self.project_local_point(pt, false), FeatureId::Unknown)
    }
}

/// A point of a shape that may touch the field: where it is (the field's space), how far
/// the shape's surface there is from the field's (negative: in it), and which way is out.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Probe {
    /// The point on the shape's surface nearest the field, in the field's space.
    pub point: Vector,
    /// The distance from the shape's surface to the field's surface (negative: overlap).
    pub dist: Real,
    /// Out of the field at the point (unit).
    pub normal: Vector,
    /// Which probe this is, for matching contacts between steps.
    pub id: u32,
}

impl Probe {
    /// The matching point on the field's surface.
    pub fn surface_point(&self) -> Vector {
        self.point - self.normal * self.dist
    }
}

/// Samples along a capsule's segment, before refining the nearest.
const CAPSULE_SAMPLES: usize = 5;
/// Golden-section steps refining the nearest point of a segment.
const REFINE_STEPS: usize = 8;
/// Steps of the search for a convex shape's deepest point.
const DEEPEST_STEPS: usize = 4;
/// Probe ids of a compound's parts are its part's index times this plus the part's own
/// (a part has fewer probes than this).
const PART_IDS: u32 = 64;

/// The points of `shape`, placed at `pose` in the field's space, that may touch `field`,
/// nearest first; `None` for shapes this can't probe (balls, capsules, shapes with a
/// support map, triangle meshes, and compounds of these). With `toward` (a direction of
/// motion, field's space), also the shape's side facing it, which may meet the field
/// first.
///
/// Probe ids (for matching contacts between steps): a capsule's samples along its segment
/// `0..CAPSULE_SAMPLES`, then its nearest point; a polyhedron's deepest point 0, the
/// corners of its face toward the field `1..=4` and of its face toward `toward` `5..=8`;
/// another convex shape's deepest point 0 and its point toward `toward` 9; a triangle
/// mesh's vertices by index (every vertex is probed: a mesh is a concave thing, a dug
/// chunk, with no deepest point to find); a compound's parts' own, each part's offset by
/// its index times [`PART_IDS`].
pub fn probes(
    field: &dyn DistanceField,
    pose: &Pose,
    shape: &dyn Shape,
    toward: Option<Vector>,
) -> Option<Vec<Probe>> {
    let mut out = Vec::new();
    let toward = toward.and_then(|v| v.try_normalize());
    if let Some(compound) = shape.as_shape::<Compound>() {
        for (i, (part_pose, part)) in compound.shapes().iter().enumerate() {
            let placed = *pose * *part_pose;
            for mut probe in probes(field, &placed, part.as_ref(), toward)? {
                probe.id += i as u32 * PART_IDS;
                out.push(probe);
            }
        }
    } else if let Some(mesh) = shape.as_shape::<TriMesh>() {
        for (i, v) in mesh.vertices().iter().enumerate() {
            out.push(point_probe(field, pose.transform_point(*v), 0.0, i as u32));
        }
    } else if let Some(ball) = shape.as_shape::<Ball>() {
        out.push(sphere_probe(field, pose.translation, ball.radius, 0));
    } else if let Some(capsule) = shape.as_shape::<Capsule>() {
        capsule_probes(field, pose, capsule, &mut out);
    } else if let Some((map, border)) = shape.as_polygonal_feature_map() {
        // The deepest point, then the face (or edge) of the shape toward the field: a box
        // resting flat touches at its four corners.
        let deepest = deepest_support(field, pose, |dir| map.local_support_point(dir), border);
        let mut faces = vec![(-deepest.normal, 1)];
        if let Some(dir) = toward {
            faces.push((dir, 5));
        }
        out.push(deepest);
        for (dir, first) in faces {
            let mut feature = PolygonalFeature::default();
            map.local_support_feature(pose.rotation.inverse() * dir, &mut feature);
            for (k, &v) in feature.vertices[..feature.num_vertices].iter().enumerate() {
                out.push(point_probe(
                    field,
                    pose.transform_point(v),
                    border,
                    first + k as u32,
                ));
            }
        }
    } else {
        let map = shape.as_support_map()?;
        out.push(deepest_support(
            field,
            pose,
            |dir| map.local_support_point(dir),
            0.0,
        ));
        if let Some(dir) = toward {
            let local = map.local_support_point(pose.rotation.inverse() * dir);
            out.push(point_probe(field, pose.transform_point(local), 0.0, 9));
        }
    }
    out.sort_by(|a, b| a.dist.total_cmp(&b.dist));
    Some(out)
}

/// The point `p` of a shape (inside its rounded border of `border`, if any).
fn point_probe(field: &dyn DistanceField, p: Vector, border: Real, id: u32) -> Probe {
    let (d, normal) = field.sample(p);
    Probe {
        point: p - normal * border,
        dist: d - border,
        normal,
        id,
    }
}

/// A sphere of `radius` at `center`.
fn sphere_probe(field: &dyn DistanceField, center: Vector, radius: Real, id: u32) -> Probe {
    let (d, normal) = field.sample(center);
    Probe {
        point: center - normal * radius,
        dist: d - radius,
        normal,
        id,
    }
}

/// A capsule: spheres along its segment (each finds what is nearest it: the ground under
/// one end, a wall beside its middle), and the nearest of them refined (a corner between
/// samples is found).
fn capsule_probes(field: &dyn DistanceField, pose: &Pose, capsule: &Capsule, out: &mut Vec<Probe>) {
    let a = pose.transform_point(capsule.segment.a);
    let b = pose.transform_point(capsule.segment.b);
    let r = capsule.radius;
    let at = |s: Real| a + (b - a) * s;
    let d = |s: Real| field.distance(at(s));
    let n = CAPSULE_SAMPLES;
    let s = |i: usize| i as Real / (n - 1) as Real;
    let samples: Vec<Real> = (0..n).map(|i| d(s(i))).collect();
    for i in 0..n {
        out.push(sphere_probe(field, at(s(i)), r, i as u32));
    }
    let k = (0..n)
        .min_by(|&i, &j| samples[i].total_cmp(&samples[j]))
        .unwrap_or(0);
    // Golden-section search between the nearest sample's neighbors.
    let step = 1.0 / (n - 1) as Real;
    let (mut lo, mut hi) = (
        (k as Real - 1.0).max(0.0) * step,
        (k as Real + 1.0).min((n - 1) as Real) * step,
    );
    let g = 0.618_034_f64 as Real;
    let (mut x1, mut x2) = (hi - g * (hi - lo), lo + g * (hi - lo));
    let (mut f1, mut f2) = (d(x1), d(x2));
    for _ in 0..REFINE_STEPS {
        if f1 < f2 {
            hi = x2;
            x2 = x1;
            f2 = f1;
            x1 = hi - g * (hi - lo);
            f1 = d(x1);
        } else {
            lo = x1;
            x1 = x2;
            f1 = f2;
            x2 = lo + g * (hi - lo);
            f2 = d(x2);
        }
    }
    let best = if f1 < f2 { x1 } else { x2 };
    out.push(sphere_probe(field, at(best), r, n as u32));
}

/// A convex shape's deepest point toward the field: its support point away from the
/// field's normal, the normal taken again there, a few times.
fn deepest_support(
    field: &dyn DistanceField,
    pose: &Pose,
    support: impl Fn(Vector) -> Vector,
    border: Real,
) -> Probe {
    let (_, mut normal) = field.sample(pose.translation);
    if normal == Vector::ZERO {
        normal = Vector::Y;
    }
    let mut p = pose.transform_point(support(pose.rotation.inverse() * -normal));
    let (mut d, mut at_p) = field.sample(p);
    for _ in 0..DEEPEST_STEPS {
        if at_p == Vector::ZERO || at_p.dot(normal) > 0.9999 {
            break;
        }
        normal = at_p;
        p = pose.transform_point(support(pose.rotation.inverse() * -normal));
        (d, at_p) = field.sample(p);
    }
    let normal = at_p.try_normalize().unwrap_or(normal);
    Probe {
        point: p - normal * border,
        dist: d - border,
        normal,
        id: 0,
    }
}

#[cfg(test)]
pub(crate) mod tests {
    use super::*;

    /// Ground below y = 0.
    pub struct Plane;
    impl DistanceField for Plane {
        fn distance(&self, p: Vector) -> Real {
            p.y
        }
    }

    /// Ground below y = 0 with a unit box sitting on it at the origin (x and z in
    /// [-0.5, 0.5], y up to 1): a step.
    pub struct Step;
    impl DistanceField for Step {
        fn distance(&self, p: Vector) -> Real {
            let q = Vector::new(p.x.abs() - 0.5, (p.y - 0.5).abs() - 0.5, p.z.abs() - 0.5);
            let outside = q.max(Vector::ZERO).length() + q.max_element().min(0.0);
            p.y.min(outside)
        }
    }

    pub fn shape_at(field: impl DistanceField + 'static) -> SdfShape {
        SdfShape::new(0, Arc::new(field))
    }

    #[test]
    fn rays_stop_on_the_surface() {
        let shape = shape_at(Plane);
        let ray = Ray::new(Vector::new(1.0, 10.0, 2.0), Vector::new(0.0, -2.0, 0.0));
        let hit = shape
            .cast_local_ray_and_get_normal(&ray, 100.0, true)
            .expect("hits the ground");
        assert!(
            (hit.time_of_impact - 5.0).abs() < 1.0e-2,
            "{}",
            hit.time_of_impact
        );
        assert!(hit.normal.distance(Vector::Y) < 1.0e-3);
        assert!(
            shape
                .cast_local_ray_and_get_normal(&ray, 4.0, true)
                .is_none()
        );
        // Upward: never.
        let up = Ray::new(Vector::new(0.0, 1.0, 0.0), Vector::Y);
        assert!(
            shape
                .cast_local_ray_and_get_normal(&up, 50.0, true)
                .is_none()
        );
    }

    #[test]
    fn points_project_onto_the_surface() {
        let shape = shape_at(Plane);
        let proj = shape.project_local_point(Vector::new(1.0, 3.0, 0.0), true);
        assert!(!proj.is_inside);
        assert!(proj.point.distance(Vector::new(1.0, 0.0, 0.0)) < 1.0e-3);
        assert!(
            shape
                .project_local_point(Vector::new(0.0, -1.0, 0.0), true)
                .is_inside
        );
    }

    #[test]
    fn a_ball_probes_its_lowest_point() {
        let ball = Ball::new(0.5);
        let probes = probes(&Plane, &Pose::translation(0.0, 0.4, 0.0), &ball, None).unwrap();
        assert_eq!(probes.len(), 1);
        assert!((probes[0].dist + 0.1).abs() < 1.0e-4);
        assert!(probes[0].point.distance(Vector::new(0.0, -0.1, 0.0)) < 1.0e-3);
    }

    #[test]
    fn a_capsule_finds_a_corner_between_its_samples() {
        // Lying along x at height 1.2, its segment from x = -1.3 to 1.3: the step's top
        // edge (y = 1) is nearest under its middle.
        let capsule = Capsule::new_x(1.3, 0.3);
        let lying = probes(&Step, &Pose::translation(0.0, 1.2, 0.0), &capsule, None).unwrap();
        // The top is 0.2 below the axis: 0.1 into the capsule.
        assert!((lying[0].dist + 0.1).abs() < 1.0e-2, "{}", lying[0].dist);
        // Standing beside the step.
        let standing = Capsule::new_y(0.7, 0.3);
        let beside = probes(&Step, &Pose::translation(0.75, 1.05, 0.0), &standing, None).unwrap();
        // Beside the step's side (x = 0.5), 0.25 from it: 0.05 into the capsule.
        assert!((beside[0].dist + 0.05).abs() < 1.0e-2, "{beside:?}");
    }

    #[test]
    fn a_compound_probes_each_part_in_place() {
        // Two balls side by side, the second lower: it touches first.
        let compound = Compound::new(vec![
            (Pose::translation(-1.0, 0.0, 0.0), SharedShape::ball(0.5)),
            (Pose::translation(1.0, -0.2, 0.0), SharedShape::ball(0.5)),
        ]);
        let probes = probes(&Plane, &Pose::translation(0.0, 0.6, 0.0), &compound, None).unwrap();
        assert_eq!(probes.len(), 2);
        // Nearest first: the lower ball, 0.1 into the ground, its id the second part's.
        assert!((probes[0].dist + 0.1).abs() < 1.0e-4, "{probes:?}");
        assert_eq!(probes[0].id, PART_IDS);
        assert!(probes[0].point.distance(Vector::new(1.0, -0.1, 0.0)) < 1.0e-3);
        assert!((probes[1].dist - 0.1).abs() < 1.0e-4);
        assert_eq!(probes[1].id, 0);
    }

    #[test]
    fn a_mesh_probes_its_vertices() {
        // A tetrahedron with one vertex down, 0.1 into the ground.
        let mesh = TriMesh::new(
            vec![
                Vector::new(0.0, -0.1, 0.0),
                Vector::new(1.0, 1.0, 0.0),
                Vector::new(-1.0, 1.0, 0.0),
                Vector::new(0.0, 1.0, 1.0),
            ],
            vec![[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]],
        )
        .unwrap();
        let probes = probes(&Plane, &Pose::IDENTITY, &mesh, None).unwrap();
        assert_eq!(probes.len(), 4);
        assert!((probes[0].dist + 0.1).abs() < 1.0e-4, "{probes:?}");
        assert_eq!(probes[0].id, 0);
        assert!(probes[1..].iter().all(|p| (p.dist - 1.0).abs() < 1.0e-4));
    }

    #[test]
    fn a_box_resting_flat_touches_at_its_corners() {
        let cuboid = Cuboid::new(Vector::new(0.5, 0.25, 0.5));
        let probes = probes(&Plane, &Pose::translation(3.0, 0.24, 0.0), &cuboid, None).unwrap();
        let touching = probes.iter().filter(|p| p.dist < 0.0).count();
        // The deepest point and the four corners of the bottom face.
        assert!(touching >= 4, "{probes:?}");
        assert!(probes.iter().all(|p| (p.dist + 0.01).abs() < 1.0e-3));
    }
}
