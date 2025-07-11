use wasm_bindgen::prelude::*;
use num_dual::*;
use nalgebra::{SVector, Vector2};

// x, y, theta, a, vx, vy, wx, wy
const NUM_STATES: usize = 8;

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

fn state_transition<D: DualNum<f32> + Copy>(input: SVector<D, 11>) -> SVector<D, NUM_STATES> {
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

    // scale the boat direction by the throttle
    let engine_force = boat_direction * (throttle.clone() * rudder.sin());
    force += engine_force;

    boat_direction.dot(&wind_velocity);
    

    todo!()
}


#[cfg(test)]
mod tests {
    use super::*;

    
}
