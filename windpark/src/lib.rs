mod kalman_filter;
mod lsm_mode;
mod physics;
mod rampage;

use std::f32;

use nalgebra::{Rotation2, SVector, Vector2};
use num_dual::*;
use wasm_bindgen::prelude::*;

// x, y, theta, a, vx, vy, wx, wy
const NUM_STATES: usize = 8;

type Float = f32;

#[wasm_bindgen]
pub struct Windpark {
    x: [Float; NUM_STATES],
    motorboat_dynamics: physics::motorboat_model::MotorboatModel,
}

#[wasm_bindgen]
impl Windpark {
    #[wasm_bindgen(constructor)]
    pub fn new() -> Windpark {
        Windpark {
            x: [0.0; NUM_STATES],
            motorboat_dynamics: rampage::create_motorboat_model(),
        }
    }

    #[wasm_bindgen]
    pub fn step(&mut self, rudder: Float, throttle: Float, dt: Float) {
        let x_hat = physics::motorboat_model::state_transition(
            &self.motorboat_dynamics,
            dt,
            rudder,
            throttle,
            self.x.into(),
        );
        self.x = x_hat.into();
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
    pub fn angular_velocity(&self) -> Float {
        self.x[3]
    }

    #[wasm_bindgen]
    pub fn speed(&self) -> Float {
        let speed = Vector2::new(self.x[4], self.x[5]).norm();
        if speed.is_nan() { 0.0 } else { speed }
    }

    #[wasm_bindgen]
    pub fn velocity(&self) -> Vec<Float> {
        vec![self.x[4], self.x[5]]
    }

    pub fn get_state(&self) -> Vec<Float> {
        self.x.to_vec()
    }
}

fn predict_measurement<D: DualNum<f32> + Copy + nalgebra::RealField>(
    x: SVector<D, NUM_STATES>,
) -> SVector<D, 3> {
    SVector::from([
        x[0], x[1], x[2], // theta
    ])
}

// fn state_transition<D: DualNum<f32> + Copy + nalgebra::RealField>(
//     input: SVector<D, 11>,
// ) -> SVector<D, NUM_STATES> {
//     let dt = &input[0];
//     let rudder = &input[1];
//     let throttle = &input[2];

//     let mut pos = Vector2::new(input[3].clone(), input[4].clone());
//     let mut theta = input[5].clone();
//     let mut angular_velocity = input[6].clone();
//     let mut velocity = Vector2::new(input[7].clone(), input[8].clone());
//     let wind_velocity = Vector2::new(input[9].clone(), input[10].clone());

//     let boat_direction = Vector2::new(theta.cos(), theta.sin());

//     let mut torque = D::zero();
//     let mut force = Vector2::zeros();

//     // add engine force
//     let rudder_rotation = Rotation2::new(-rudder.clone());
//     force += (rudder_rotation * boat_direction.clone()) * (throttle.clone() * THRUST_FORCE);
//     torque += rudder.sin() * throttle.clone() * THRUST_TORQUE_COEFFICIENT;

//     let speed = velocity.norm();
//     if speed > 0.001.into() {
//         // calculate lift and drag forces from keel
//         let dynamic_pressure = velocity.norm_squared() * 0.5 * WATER_DENSITY;

//         // let a = boat_direction - velocity.normalize();
//         // let angle_of_attack = a[1].atan2(a[0]);

//         let mut angle_of_attack = boat_direction.angle(&velocity);
//         if boat_direction.perp(&velocity.normalize()) < D::zero() {
//             angle_of_attack = -angle_of_attack;
//         }

//         // let cl = D::from(LIFT_COEFFICIENT) * (angle_of_attack * 2.0).sin();
//         let cd = D::from(0.05);
//         let cl = D::from(0.07) * angle_of_attack;

//         let lift = dynamic_pressure * cl * KEEL_AREA;
//         let drag = dynamic_pressure * cd * KEEL_AREA;

//         let rotate90 = Rotation2::new(D::from(-f32::consts::FRAC_PI_2));
//         force += (rotate90 * boat_direction) * lift;

//         force -= velocity.normalize() * drag;
//     }

//     let angular_damping = (angular_velocity * angular_velocity) * 200.0;
//     if angular_velocity > D::zero() {
//         torque -= angular_damping;
//     } else if angular_velocity < D::zero() {
//         torque += angular_damping;
//     }

//     // calculate apparent wind
//     let apparent_wind = wind_velocity - velocity;
//     let dynamic_pressure = apparent_wind.norm_squared() * 0.5 * AIR_DENSITY * HULL_AREA;
//     let mut angle_of_attack = boat_direction.angle(&apparent_wind);
//     if boat_direction.perp(&apparent_wind.normalize()) < D::zero() {
//         angle_of_attack = -angle_of_attack;
//     }
//     force += wind_velocity.normalize() * dynamic_pressure;
//     torque += angle_of_attack.sin() * dynamic_pressure;

//     velocity += force * (dt.clone() / BOAT_MASS);
//     pos += velocity * dt.clone();

//     angular_velocity += torque * (dt.clone() / MOMENT_OF_INERTIA);
//     theta += angular_velocity * dt.clone();

//     SVector::from([
//         pos[0],
//         pos[1],
//         theta,
//         angular_velocity,
//         velocity[0],
//         velocity[1],
//         wind_velocity[0],
//         wind_velocity[1],
//     ])
// }

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
