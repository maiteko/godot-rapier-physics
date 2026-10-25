use godot::classes::physics_server_3d::ShapeType;
use godot::prelude::*;

use super::rapier_shape::RapierShape;
use crate::rapier_wrapper::prelude::*;
use crate::rapier_wrapper::sdf;
use crate::rapier_wrapper::sdf::SdfShape;
use crate::servers::rapier_physics_singleton::PhysicsShapes;
use crate::servers::rapier_physics_singleton::RapierId;
use crate::shapes::rapier_shape::IRapierShape;
use crate::shapes::rapier_shape_base::RapierShapeBase;

/// Godot's custom shape: a signed distance field ([`SdfShape`]). Its data is the key of a
/// field registered with [`sdf::register_field`] (an int).
pub struct RapierSdfShape {
    key: i64,
    base: RapierShapeBase,
}

impl RapierSdfShape {
    pub fn create(id: RapierId, rid: Rid, physics_shapes: &mut PhysicsShapes) {
        let shape = Self {
            key: 0,
            base: RapierShapeBase::new(id, rid),
        };
        physics_shapes.insert(rid, RapierShape::RapierSdfShape(shape));
    }
}

impl IRapierShape for RapierSdfShape {
    fn get_base(&self) -> &RapierShapeBase {
        &self.base
    }

    fn get_mut_base(&mut self) -> &mut RapierShapeBase {
        &mut self.base
    }

    fn get_type(&self) -> ShapeType {
        ShapeType::CUSTOM
    }

    fn allows_one_way_collision(&self) -> bool {
        false
    }

    fn set_data(&mut self, data: Variant, physics_engine: &mut PhysicsEngine) {
        let Ok(key) = data.try_to::<i64>() else {
            godot_error!("A custom (SDF) shape's data must be a field key (int). Got {data}");
            return;
        };
        let Some(field) = sdf::field(key) else {
            godot_error!("No distance field is registered as {key}");
            return;
        };
        self.key = key;
        physics_engine.insert_shape(
            rapier::prelude::SharedShape::new(SdfShape::new(key, field)),
            self.base.get_id(),
        );
        self.base.reset_aabb(physics_engine);
    }

    fn get_data(&self, _physics_engine: &PhysicsEngine) -> Variant {
        self.key.to_variant()
    }
}
