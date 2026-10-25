#[cfg(all(feature = "single", feature = "dim2"))]
extern crate rapier2d as rapier;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate rapier2d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate rapier3d as rapier;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate rapier3d_f64 as rapier;
#[cfg(all(feature = "single", feature = "dim2"))]
extern crate salva2d as salva;
#[cfg(all(feature = "double", feature = "dim2"))]
extern crate salva2d_f64 as salva;
#[cfg(all(feature = "single", feature = "dim3"))]
extern crate salva3d as salva;
#[cfg(all(feature = "double", feature = "dim3"))]
extern crate salva3d_f64 as salva;
pub mod bodies;
pub mod fluids;
mod joints;
pub mod nodes;
mod rapier_wrapper;
pub mod servers;
mod shapes;
pub mod spaces;
mod types;
use godot::prelude::*;
// With `embedded`, the library has no entry point of its own: another extension links
// it and calls `servers::register_server` (servers stage) and `servers::register_scene`
// (scene stage) from its own `ExtensionLibrary`.
#[cfg(all(feature = "dim2", not(feature = "embedded")))]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 2D extension library.
pub struct RapierPhysics2DExtensionLibrary {}
#[cfg(all(feature = "dim2", not(feature = "embedded")))]
#[gdextension(entry_symbol = rapier_2d_init)]
unsafe impl ExtensionLibrary for RapierPhysics2DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_stage_deinit(_level: InitStage) {}
}
#[cfg(all(feature = "dim3", not(feature = "embedded")))]
#[derive(GodotClass)]
#[class(base=Object, init)]
/// Used to register the Rapier 3D extension library.
pub struct RapierPhysics3DExtensionLibrary {}
#[cfg(all(feature = "dim3", not(feature = "embedded")))]
#[gdextension(entry_symbol = rapier_3d_init)]
unsafe impl ExtensionLibrary for RapierPhysics3DExtensionLibrary {
    fn min_level() -> InitLevel {
        InitLevel::Servers
    }

    fn on_stage_init(level: InitStage) {
        match level {
            InitStage::Scene => {
                servers::register_scene();
            }
            InitStage::Servers => {
                servers::register_server();
            }
            _ => (),
        }
    }

    fn on_stage_deinit(_level: InitStage) {}
}
