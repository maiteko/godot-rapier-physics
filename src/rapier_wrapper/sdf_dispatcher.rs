//! Queries between an [`SdfShape`] and other shapes, from [`probes`]: the first link of the
//! narrow phase's dispatcher chain. Pairs without an SDF shape are `Unsupported`, for the
//! rest of the chain.

use rapier::parry::query::ClosestPoints;
use rapier::parry::query::Contact;
use rapier::parry::query::ContactManifold as ParryContactManifold;
use rapier::parry::query::ContactManifoldsWorkspace;
use rapier::parry::query::NonlinearRigidMotion;
use rapier::parry::query::PersistentQueryDispatcher;
use rapier::parry::query::QueryDispatcher;
use rapier::parry::query::ShapeCastHit;
use rapier::parry::query::ShapeCastOptions;
use rapier::parry::query::ShapeCastStatus;
use rapier::parry::query::ShapeDistance;
use rapier::parry::query::ShapeIntersection;
use rapier::parry::query::Unsupported;
use rapier::parry::query::details::NormalConstraints;
use rapier::parry::shape::PackedFeatureId;
use rapier::parry::shape::Shape;
use rapier::prelude::*;

use super::sdf::DistanceField;
use super::sdf::GRADIENT_STEP;
use super::sdf::Probe;
use super::sdf::SdfShape;
use super::sdf::probes;

/// Answers the queries involving an [`SdfShape`].
#[derive(Copy, Clone, Debug, Default)]
pub struct SdfDispatcher;

/// Most conservative-advancement steps of a shape cast.
const MAX_CAST_STEPS: usize = 256;
/// The least a shape cast advances past what it isn't approaching (a floor it slides
/// along): a surface rising out of such a floor is met at most this far in.
const MIN_CAST_STEP: Real = 0.05;
/// Approaching (along the normal) slower than this fraction of the speed is sliding along.
const APPROACHING: Real = 1.0e-3;
/// Nearer than this (beyond the target distance), a cast has hit.
const CAST_TOLERANCE: Real = 1.0e-3;
/// Shape casts advance by this fraction of the distance (the field may be a little steeper
/// than a true distance).
const CAST_SAFETY: Real = 0.9;

/// Which of the pair is the field, with the other shape's pose in the field's space.
enum Pair<'a> {
    /// The first shape is the field.
    FieldFirst(&'a SdfShape, Pose, &'a dyn Shape),
    /// The second shape is the field.
    FieldSecond(&'a SdfShape, Pose, &'a dyn Shape),
}

/// The pair, if exactly one of them is an SDF shape; `Err` when neither is (for the rest
/// of the chain), `Ok(None)` when both are (fields don't collide).
fn pair<'a>(
    pos12: &Pose,
    g1: &'a dyn Shape,
    g2: &'a dyn Shape,
) -> Result<Option<Pair<'a>>, Unsupported> {
    match (g1.as_shape::<SdfShape>(), g2.as_shape::<SdfShape>()) {
        (Some(_), Some(_)) => Ok(None),
        (Some(f), None) => Ok(Some(Pair::FieldFirst(f, *pos12, g2))),
        (None, Some(f)) => Ok(Some(Pair::FieldSecond(f, pos12.inverse(), g1))),
        (None, None) => Err(Unsupported),
    }
}

impl Pair<'_> {
    fn field(&self) -> &dyn DistanceField {
        match self {
            Pair::FieldFirst(f, ..) | Pair::FieldSecond(f, ..) => f.field.as_ref(),
        }
    }

    /// The other shape's probes, nearest first.
    fn probes(&self) -> Result<Vec<Probe>, Unsupported> {
        let (pose, shape) = match self {
            Pair::FieldFirst(_, pose, shape) | Pair::FieldSecond(_, pose, shape) => (pose, *shape),
        };
        probes(self.field(), pose, shape, None).ok_or(Unsupported)
    }

    fn pose(&self) -> &Pose {
        match self {
            Pair::FieldFirst(_, pose, _) | Pair::FieldSecond(_, pose, _) => pose,
        }
    }

    /// `probe` as a contact between the pair's first and second shapes (each point and
    /// normal in its own shape's space).
    fn contact(&self, probe: &Probe) -> Contact {
        let pose = self.pose();
        let on_shape = pose.inverse_transform_point(probe.point);
        let shape_normal = -(pose.rotation.inverse() * probe.normal);
        let on_field = probe.surface_point();
        match self {
            Pair::FieldFirst(..) => {
                Contact::new(on_field, on_shape, probe.normal, shape_normal, probe.dist)
            }
            Pair::FieldSecond(..) => {
                Contact::new(on_shape, on_field, shape_normal, probe.normal, probe.dist)
            }
        }
    }
}

impl QueryDispatcher for SdfDispatcher {
    fn intersection_test(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<ShapeIntersection, Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            return Ok(ShapeIntersection::new(false));
        };
        let probes = pair.probes()?;
        Ok(ShapeIntersection::new(
            probes.first().is_some_and(|p| p.dist < 0.0),
        ))
    }

    fn distance(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
    ) -> Result<ShapeDistance, Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            return Ok(ShapeDistance::new(Real::MAX));
        };
        let probes = pair.probes()?;
        Ok(ShapeDistance::new(
            probes.first().map_or(Real::MAX, |p| p.dist.max(0.0)),
        ))
    }

    fn contact(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: Real,
    ) -> Result<Option<Contact>, Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            return Ok(None);
        };
        let probes = pair.probes()?;
        Ok(probes
            .first()
            .filter(|p| p.dist <= prediction)
            .map(|p| pair.contact(p)))
    }

    fn closest_points(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        max_dist: Real,
    ) -> Result<ClosestPoints, Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            return Ok(ClosestPoints::Disjoint);
        };
        let probes = pair.probes()?;
        Ok(match probes.first() {
            Some(p) if p.dist < 0.0 => ClosestPoints::Intersecting,
            Some(p) if p.dist <= max_dist => {
                let c = pair.contact(p);
                ClosestPoints::WithinMargin(c.point1, c.point2)
            }
            _ => ClosestPoints::Disjoint,
        })
    }

    fn cast_shapes(
        &self,
        pos12: &Pose,
        local_vel12: Vector,
        g1: &dyn Shape,
        g2: &dyn Shape,
        options: ShapeCastOptions,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            return Ok(None);
        };
        // The other shape's velocity relative to the field, in the field's space.
        let vel = match pair {
            Pair::FieldFirst(..) => local_vel12,
            Pair::FieldSecond(..) => pos12.rotation.inverse() * -local_vel12,
        };
        let start = *pair.pose();
        let hit = advance(
            pair.field(),
            |t| Pose::from_parts(start.translation + vel * t, start.rotation),
            &pair,
            vel.length(),
            vel,
            0.0,
            options.max_time_of_impact,
            options.target_distance,
            options.stop_at_penetration,
        )?;
        Ok(hit)
    }

    fn cast_shapes_nonlinear(
        &self,
        motion1: &NonlinearRigidMotion,
        g1: &dyn Shape,
        motion2: &NonlinearRigidMotion,
        g2: &dyn Shape,
        start_time: Real,
        end_time: Real,
        stop_at_penetration: bool,
    ) -> Result<Option<ShapeCastHit>, Unsupported> {
        let pos12 = |t: Real| motion1.position_at_time(t).inverse() * motion2.position_at_time(t);
        let Some(pair) = pair(&pos12(start_time), g1, g2)? else {
            return Ok(None);
        };
        let (field_motion, shape_motion, shape) = match pair {
            Pair::FieldFirst(_, _, s) => (motion1, motion2, s),
            Pair::FieldSecond(_, _, s) => (motion2, motion1, s),
        };
        let pose_at =
            |t: Real| field_motion.position_at_time(t).inverse() * shape_motion.position_at_time(t);
        // How fast any point of the shape can approach the field.
        let sphere = shape.compute_local_bounding_sphere();
        let radius = sphere.center.length() + sphere.radius;
        let reach = pose_at(start_time).translation.length() + radius;
        let speed = (shape_motion.linvel - field_motion.linvel).length()
            + shape_motion.angvel.length() * radius
            + field_motion.angvel.length() * reach;
        let hit = advance(
            pair.field(),
            pose_at,
            &pair,
            speed,
            shape_motion.linvel - field_motion.linvel,
            start_time,
            end_time,
            0.0,
            stop_at_penetration,
        )?;
        Ok(hit)
    }
}

/// Conservative advancement: from `start` on, the other shape at `pose_at(t)` in the
/// field's space, moving no faster than `speed` (with velocity about `vel`), until a probe
/// it is approaching is within `target` of the field. Probes it slides along or leaves
/// (a floor under a walking body) don't stop it, and only limit its step to
/// [`MIN_CAST_STEP`]. `Ok(None)` when it gets to `end`.
#[allow(clippy::too_many_arguments)]
fn advance(
    field: &dyn DistanceField,
    pose_at: impl Fn(Real) -> Pose,
    pair: &Pair<'_>,
    speed: Real,
    vel: Vector,
    start: Real,
    end: Real,
    target: Real,
    stop_at_penetration: bool,
) -> Result<Option<ShapeCastHit>, Unsupported> {
    let shape = match pair {
        Pair::FieldFirst(_, _, s) | Pair::FieldSecond(_, _, s) => *s,
    };
    if speed <= 0.0 {
        return Ok(None);
    }
    let dir = vel.try_normalize();
    let approaching = |p: &Probe| dir.is_some_and(|d| -d.dot(p.normal) > APPROACHING);
    let mut t = start;
    for _ in 0..MAX_CAST_STEPS {
        let pose = pose_at(t);
        let probes = probes(field, &pose, shape, dir).ok_or(Unsupported)?;
        // Already in the field at the start: a hit there, unless leaving it.
        if t == start
            && let Some(p) = probes.first().filter(|p| p.dist - target < 0.0)
            && (stop_at_penetration || approaching(p))
        {
            let status = ShapeCastStatus::PenetratingOrWithinTargetDist;
            return Ok(Some(cast_hit(pair, &pose, p, t, status)));
        }
        let touching = |p: &Probe, within: Real| approaching(p) && p.dist - target <= within;
        if probes.iter().any(|p| touching(p, CAST_TOLERANCE)) {
            // The one most head-on of those about there: a wall's rather than the crease
            // between the floor and the wall, whose direction (from differences a gradient
            // step apart) is ambiguous and turns toward the wall a little early.
            let head_on = |p: &&Probe| dir.map_or(0.0, |d| -d.dot(p.normal));
            let near = CAST_TOLERANCE.max(2.0 * GRADIENT_STEP);
            if let Some(p) = probes
                .iter()
                .filter(|p| touching(p, near))
                .max_by(|a, b| head_on(a).total_cmp(&head_on(b)))
            {
                return Ok(Some(cast_hit(
                    pair,
                    &pose,
                    p,
                    t,
                    ShapeCastStatus::Converged,
                )));
            }
        }
        let step = probes
            .iter()
            .map(|p| {
                let gap = p.dist - target;
                if approaching(p) {
                    gap * CAST_SAFETY
                } else {
                    gap.max(MIN_CAST_STEP)
                }
            })
            .fold(Real::MAX, Real::min)
            .max(CAST_TOLERANCE);
        t += step / speed;
        if t > end {
            return Ok(None);
        }
    }
    Ok(None)
}

/// A shape cast's hit from `probe`, the other shape at `pose`, between the pair's first
/// and second shapes.
fn cast_hit(
    pair: &Pair<'_>,
    pose: &Pose,
    probe: &Probe,
    t: Real,
    status: ShapeCastStatus,
) -> ShapeCastHit {
    let on_shape = pose.inverse_transform_point(probe.point);
    let shape_normal = -(pose.rotation.inverse() * probe.normal);
    let hit = ShapeCastHit {
        time_of_impact: t,
        witness1: probe.surface_point(),
        witness2: on_shape,
        normal1: probe.normal,
        normal2: shape_normal,
        status,
        subshape1: 0,
        subshape2: 0,
    };
    match pair {
        Pair::FieldFirst(..) => hit,
        Pair::FieldSecond(..) => hit.swapped(),
    }
}

impl<ManifoldData, ContactData> PersistentQueryDispatcher<ManifoldData, ContactData>
    for SdfDispatcher
where
    ManifoldData: Default,
    ContactData: Default + Copy,
{
    /// One manifold per probe (each with its own normal: the field curves), kept in the
    /// probe's slot so contacts keep their warm-start data from step to step.
    fn contact_manifolds(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        prediction: Real,
        manifolds: &mut Vec<ParryContactManifold<ManifoldData, ContactData>>,
        _workspace: &mut Option<ContactManifoldsWorkspace>,
    ) -> Result<(), Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            manifolds.clear();
            return Ok(());
        };
        let probes = pair.probes()?;
        let slots = probes.iter().map(|p| p.id as usize + 1).max().unwrap_or(0);
        manifolds.resize_with(slots, ParryContactManifold::new);
        manifolds.truncate(slots);
        for manifold in manifolds.iter_mut() {
            manifold.points.clear();
        }
        for probe in probes.iter().filter(|p| p.dist <= prediction) {
            set_manifold(&pair, probe, &mut manifolds[probe.id as usize]);
        }
        Ok(())
    }

    fn contact_manifold_convex_convex(
        &self,
        pos12: &Pose,
        g1: &dyn Shape,
        g2: &dyn Shape,
        _normal_constraints1: Option<&dyn NormalConstraints>,
        _normal_constraints2: Option<&dyn NormalConstraints>,
        prediction: Real,
        manifold: &mut ParryContactManifold<ManifoldData, ContactData>,
    ) -> Result<(), Unsupported> {
        let Some(pair) = pair(pos12, g1, g2)? else {
            manifold.points.clear();
            return Ok(());
        };
        let probes = pair.probes()?;
        match probes.first().filter(|p| p.dist <= prediction) {
            Some(probe) => set_manifold(&pair, probe, manifold),
            None => manifold.points.clear(),
        }
        Ok(())
    }
}

/// `manifold` holding `probe` as its one contact, keeping the old contact's data.
fn set_manifold<ManifoldData, ContactData: Default + Copy>(
    pair: &Pair<'_>,
    probe: &Probe,
    manifold: &mut ParryContactManifold<ManifoldData, ContactData>,
) {
    let contact = pair.contact(probe);
    let (fid1, fid2) = match pair {
        Pair::FieldFirst(..) => (PackedFeatureId::face(0), PackedFeatureId::face(probe.id)),
        Pair::FieldSecond(..) => (PackedFeatureId::face(probe.id), PackedFeatureId::face(0)),
    };
    let tracked = TrackedContact::new(contact.point1, contact.point2, fid1, fid2, contact.dist);
    if let Some(existing) = manifold.points.first_mut() {
        existing.copy_geometry_from(tracked);
    } else {
        manifold.points.push(tracked);
    }
    manifold.points.truncate(1);
    manifold.local_n1 = contact.normal1;
    manifold.local_n2 = contact.normal2;
}

#[cfg(test)]
mod tests {
    use std::sync::Arc;

    use rapier::parry::query::DefaultQueryDispatcher;
    use rapier::parry::shape::Ball;

    use super::*;
    use crate::rapier_wrapper::sdf::tests::Plane;
    use crate::rapier_wrapper::sdf::tests::Step;
    use crate::rapier_wrapper::sdf::tests::shape_at;

    #[test]
    fn a_ball_cast_down_lands_on_the_ground() {
        let ground = shape_at(Plane);
        let ball = Ball::new(0.5);
        let hit = SdfDispatcher
            .cast_shapes(
                &Pose::translation(0.0, 5.0, 0.0),
                Vector::new(0.0, -1.0, 0.0),
                &ground,
                &ball,
                ShapeCastOptions::with_max_time_of_impact(10.0),
            )
            .unwrap()
            .expect("hits");
        assert!(
            (hit.time_of_impact - 4.5).abs() < 1.0e-2,
            "{}",
            hit.time_of_impact
        );
        assert!(hit.normal1.distance(Vector::Y) < 1.0e-3);
        // Field second: the same, swapped.
        let hit = SdfDispatcher
            .cast_shapes(
                &Pose::translation(0.0, -5.0, 0.0),
                Vector::new(0.0, 1.0, 0.0),
                &ball,
                &ground,
                ShapeCastOptions::with_max_time_of_impact(10.0),
            )
            .unwrap()
            .expect("hits");
        assert!(
            (hit.time_of_impact - 4.5).abs() < 1.0e-2,
            "{}",
            hit.time_of_impact
        );
        assert!(hit.normal2.distance(Vector::Y) < 1.0e-3);
        // Moving away: no hit.
        assert!(
            SdfDispatcher
                .cast_shapes(
                    &Pose::translation(0.0, 5.0, 0.0),
                    Vector::Y,
                    &ground,
                    &ball,
                    ShapeCastOptions::with_max_time_of_impact(10.0),
                )
                .unwrap()
                .is_none()
        );
    }

    #[test]
    fn a_capsule_walking_into_a_step_is_stopped_by_its_side() {
        let field = shape_at(Step);
        let capsule = Capsule::new_y(0.7, 0.3);
        // Standing on the ground (bottom at y = 0) at x = -2, walking +x: its side meets
        // the step's side at x = -0.5, so its axis stops at x = -0.8.
        let hit = SdfDispatcher
            .cast_shapes(
                &Pose::translation(-2.0, 1.0, 0.0),
                Vector::X,
                &field,
                &capsule,
                ShapeCastOptions {
                    target_distance: 0.0,
                    ..ShapeCastOptions::with_max_time_of_impact(5.0)
                },
            )
            .unwrap()
            .expect("hits the step");
        assert!(
            (hit.time_of_impact - 1.2).abs() < 2.0e-2,
            "{}",
            hit.time_of_impact
        );
        assert!(
            hit.normal1.distance(Vector::NEG_X) < 1.0e-2,
            "{}",
            hit.normal1
        );
    }

    #[test]
    fn pairs_without_a_field_go_down_the_chain() {
        let a = Ball::new(1.0);
        let b = Ball::new(1.0);
        assert!(
            SdfDispatcher
                .contact(&Pose::translation(1.0, 0.0, 0.0), &a, &b, 0.0)
                .is_err()
        );
    }

    #[test]
    fn a_ball_falls_onto_the_field_and_rests() {
        let mut bodies = RigidBodySet::new();
        let mut colliders = ColliderSet::new();
        let ground = ColliderBuilder::new(SharedShape(Arc::new(shape_at(Plane)))).build();
        colliders.insert(ground);
        let ball =
            bodies.insert(RigidBodyBuilder::dynamic().translation(Vector::new(0.0, 3.0, 0.0)));
        colliders.insert_with_parent(ColliderBuilder::ball(0.5).build(), ball, &mut bodies);
        let mut pipeline = PhysicsPipeline::new();
        let mut islands = IslandManager::new();
        let mut broad_phase = DefaultBroadPhase::new();
        let mut narrow_phase =
            NarrowPhase::with_query_dispatcher(SdfDispatcher.chain(DefaultQueryDispatcher));
        let mut impulse_joints = ImpulseJointSet::new();
        let mut multibody_joints = MultibodyJointSet::new();
        let mut soft_bodies = SoftBodySet::new();
        let mut ccd = CCDSolver::new();
        let params = IntegrationParameters::default();
        for _ in 0..240 {
            pipeline.step(
                Vector::new(0.0, -9.81, 0.0),
                &params,
                &mut islands,
                &mut broad_phase,
                &mut narrow_phase,
                &mut bodies,
                &mut colliders,
                &mut impulse_joints,
                &mut multibody_joints,
                &mut soft_bodies,
                &mut ccd,
                &(),
                &(),
            );
        }
        let y = bodies[ball].translation().y;
        assert!((y - 0.5).abs() < 0.02, "rests at {y}");
    }
}
