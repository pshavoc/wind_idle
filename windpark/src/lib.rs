use std::f32;

use wasm_bindgen::prelude::*;
use num_dual::*;
use nalgebra::{Rotation2, SVector, Vector2};

// x, y, theta, a, vx, vy, wx, wy
const NUM_STATES: usize = 8;

const WATER_DENSITY: f32 = 1000.0; // kg/m^3, typical value for water density
const AIR_DENSITY: f32 = 1.225; // kg/m^3, typical value for air density at sea level
const KEEL_AREA: f32 = 1.0; // m^2, area of the keel
const HULL_AREA: f32 = 0.5; // m^2, area of the hull exposed to wind

const LIFT_COEFFICIENT: f32 = 0.5;
const DRAG_COEFFICENT: f32 = 0.1;
const THRUST_TORQUE_COEFFICIENT: f32 = 10.0;
const THRUST_FORCE: f32 = 10.0;

const BOAT_MASS: f32 = 300.0; // mass of boat in kg
const MOMENT_OF_INERTIA: f32 = 250.0;

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
        self.x = f.into();
        
    }

    #[wasm_bindgen]
    pub fn set_wind(&mut self, wx: Float, wy: Float) {
        self.x[6] = wx;
        self.x[7] = wy;
    }

    #[wasm_bindgen]
    pub fn pos(&self) -> Vec<Float> {
        vec![self.x[0], self.x[1]]
    }

    #[wasm_bindgen]
    pub fn theta(&self) -> Float {
        self.x[2]
    }

    #[wasm_bindgen]
    pub fn speed(&self) -> Float {
        Vector2::new(self.x[4], self.x[5]).norm()
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
    let mut theta = input[5].clone();
    let mut angular_velocity = input[6].clone();
    let mut velocity = Vector2::new(input[7].clone(), input[8].clone());
    let wind_velocity = Vector2::new(input[9].clone(), input[10].clone());
    
    let boat_direction = Vector2::new(theta.cos(), theta.sin());

    let mut torque = D::zero();
    let mut force = Vector2::zeros();

    // add engine force
    let rudder_rotation = Rotation2::new(-rudder.clone());
    force += (rudder_rotation * boat_direction.clone()) * (throttle.clone() * THRUST_FORCE);
    torque += rudder.sin() * throttle.clone() * THRUST_TORQUE_COEFFICIENT;

    let speed = velocity.norm();
    if speed > 0.001.into() {
        // calculate lift and drag forces from keel
        let dynamic_pressure = velocity.norm_squared() * 0.5 * WATER_DENSITY;

        // let a = boat_direction - velocity.normalize();
        // let angle_of_attack = a[1].atan2(a[0]);

        let mut angle_of_attack = boat_direction.angle(&velocity);
        if boat_direction.perp(&velocity.normalize()) < D::zero() {
            // if the boat is moving backwards, reverse the angle of attack
            angle_of_attack = -angle_of_attack;
        }

        // let cl = D::from(LIFT_COEFFICIENT) * (angle_of_attack * 2.0).sin();
        let cd = D::from(0.05);
        let cl = D::from(0.07) * angle_of_attack;

        let lift = dynamic_pressure * cl * KEEL_AREA;
        let drag = dynamic_pressure * cd * KEEL_AREA;

        let rotate90 = Rotation2::new(D::from(-f32::consts::FRAC_PI_2));
        force += (rotate90 * boat_direction) * lift;

        force -= velocity.normalize() * drag;

    }


    let angular_damping = (angular_velocity * angular_velocity) * 200.0;
    if angular_velocity > D::zero() {
        torque -= angular_damping;
    } else if angular_velocity < D::zero() {
        torque += angular_damping;
    }

    // calculate apparent wind
    let apparent_wind = wind_velocity - velocity;
    let dynamic_pressure = apparent_wind.norm_squared() * 0.5 * AIR_DENSITY * HULL_AREA;
    let angle_of_attack = boat_direction.angle(&apparent_wind);
    // force += wind_velocity.normalize() * dynamic_pressure;

    velocity += force * (dt.clone() / BOAT_MASS);
    pos += velocity * dt.clone();

    angular_velocity += torque * (dt.clone() / MOMENT_OF_INERTIA);
    theta += angular_velocity * dt.clone();

    

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

    #[test]
    fn test_step() {
        let mut windpark = Windpark::new();
        windpark.step(0.0, 0.0, 0.05);
        let state = windpark.get_state();
        assert_eq!(state.len(), NUM_STATES);
    }

    
}
