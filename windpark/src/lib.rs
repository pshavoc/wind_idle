use std::f32;

use wasm_bindgen::prelude::*;
use num_dual::*;
use nalgebra::{Rotation2, SVector, Vector2};

// x, y, theta, a, vx, vy, wx, wy
const NUM_STATES: usize = 8;

const WATER_DENSITY: f32 = 1000.0; // kg/m^3, typical value for water density
const KEEL_AREA: f32 = 0.08; // m^2, area of the keel

const LIFT_COEFFICIENT: f32 = 1.0;
const DRAG_COEFFICENT: f32 = 0.7;

const BOAT_MASS: f32 = 300.0; // mass of boat in kg

type Float = f32;

#[wasm_bindgen]
pub struct Windpark {
    x: [Float; NUM_STATES],
}

#[wasm_bindgen]
impl Windpark {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Windpark {
        Windpark {
            x: [0.0; NUM_STATES],
        }
    }

    #[wasm_bindgen]
    pub fn step(&mut self, rudder: Float, throttle: Float, dt: Float) {

        let input = SVector::<Float, 11>::from_fn(|i, _| {
            match i {
                0 => dt,
                1 => rudder,
                2 => throttle,
                _ => self.x[i - 3],
            }
        });

        let (f, jac) = jacobian(state_transition, input);
        
    }

    pub fn get_state(&self) -> Vec<Float> {
        self.x.to_vec()
    }
}

fn state_transition<D: DualNum<f32> + Copy + nalgebra::RealField>(input: SVector<D, 11>) -> SVector<D, NUM_STATES> {
    let dt = &input[0];
    let rudder = &input[1];
    let throttle = &input[2];

    let mut pos = Vector2::new(input[3].clone(), input[4].clone());
    let theta = input[5].clone();
    let angular_velocity = input[6].clone();
    let mut velocity = Vector2::new(input[7].clone(), input[8].clone());
    let wind_velocity = Vector2::new(input[9].clone(), input[10].clone());

    let boat_direction = Vector2::new(theta.cos(), theta.sin());

    let mut force = Vector2::<D>::new(D::zero(), D::zero());

    // add engine force
    let rudder_rotation = Rotation2::new(rudder.clone());
    force += (rudder_rotation * boat_direction.clone()) * throttle.clone();

    // calculate lift and drag forces from keel
    let dynamic_pressure = velocity.norm_squared() * 0.5 * WATER_DENSITY * KEEL_AREA;
    let angle_of_attack = boat_direction.angle(&velocity);

    let drag = dynamic_pressure * ((D::one() - (angle_of_attack * 2.0).cos()) * DRAG_COEFFICENT + D::one());
    let lift = dynamic_pressure * (angle_of_attack * 2.0).sin() * LIFT_COEFFICIENT;

    force += boat_direction * -D::one() * drag;

    let rotate90 = Rotation2::new(D::from(f32::consts::FRAC_PI_2));
    force += (rotate90 * boat_direction.clone()) * lift;

    velocity += force * (dt.clone() / BOAT_MASS);
    pos += velocity * dt.clone();

    SVector::from([
        pos[0],
        pos[1],
        theta,
        angular_velocity,
        velocity[0],
        velocity[1],
        wind_velocity[0],
        wind_velocity[1],
    ])

}


#[cfg(test)]
mod tests {
    use super::*;

    
}
