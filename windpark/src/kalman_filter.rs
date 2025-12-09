use nalgebra::{SVector, Vector2};
use num_dual::jacobian;

use wasm_bindgen::prelude::*;

type CovarianceMatrix = nalgebra::SMatrix<
    Float,
    { physics::motorboat_model::NUM_STATES },
    { physics::motorboat_model::NUM_STATES },
>;
type HMatrix = nalgebra::SMatrix<Float, 2, { physics::motorboat_model::NUM_STATES }>;

use crate::{Float, physics};

#[wasm_bindgen]
pub struct MotorboatDynamicsKalmanFilter {
    model: physics::motorboat_model::MotorboatModel,
    state_estimate: [Float; physics::motorboat_model::NUM_STATES],
    covariance_estimate: CovarianceMatrix,
    process_noise: CovarianceMatrix,
    last_rudder: Float,
    last_throttle: Float,
    dt: Float,
}

#[wasm_bindgen]
impl MotorboatDynamicsKalmanFilter {

    
    fn new(dt: Float, model: physics::motorboat_model::MotorboatModel) -> Self {
        Self {
            model,
            state_estimate: [0.0; physics::motorboat_model::NUM_STATES],
            covariance_estimate: CovarianceMatrix::identity(),
            process_noise: CovarianceMatrix::identity() * 0.01,
            last_rudder: 0.0,
            last_throttle: 0.0,
            dt,
        }
    }

    #[wasm_bindgen(constructor)]
    pub fn create() -> Self {
        Self::new(
            0.1,
            crate::rampage::create_motorboat_model(),
        )
    }

    #[wasm_bindgen]
    pub fn pos(&self) -> Vec<Float> {
        vec![self.state_estimate[0], self.state_estimate[1]]
    }

    #[wasm_bindgen]
    pub fn theta(&self) -> Float {
        self.state_estimate[2]
    }

    #[wasm_bindgen]
    pub fn angular_velocity(&self) -> Float {
        self.state_estimate[3]
    }

    #[wasm_bindgen]
    pub fn speed(&self) -> Float {
        let speed = Vector2::new(self.state_estimate[4], self.state_estimate[5]).norm();
        if speed.is_nan() { 0.0 } else { speed }
    }

    #[wasm_bindgen]
    pub fn velocity(&self) -> Vec<Float> {
        vec![self.state_estimate[4], self.state_estimate[5]]
    }

    #[wasm_bindgen]
    pub fn predict(&mut self, rudder: Float, throttle: Float) {
        self.last_rudder = rudder;
        self.last_throttle = throttle;

        let f = |x| {
            physics::motorboat_model::state_transition(&self.model, self.dt, rudder, throttle, x)
        };

        let (x, jac) = jacobian(f, self.state_estimate.into());

        self.state_estimate = x.into();

        self.covariance_estimate =
            jac * self.covariance_estimate * jac.transpose() + self.process_noise;

        // let x_hat = physics::motorboat_model::state_transition(
        //     &self.model,
        //     dt,
        //     rudder,
        //     throttle,
        //     self.state_estimate.into(),
        // );

        // self.state_estimate = x_hat.into();
    }

    #[allow(non_snake_case)]
    #[wasm_bindgen]
    pub fn update_gps(&mut self, gps_position_x: Float, gps_position_y: Float, gps_position_variance: Float) {
        let z = SVector::<Float, 2>::from_row_slice(&[gps_position_x, gps_position_y]);
        let R = nalgebra::Matrix2::<Float>::identity() * gps_position_variance.abs();

        let y = z - h(&self.state_estimate);

        let f = |x| {
            physics::motorboat_model::state_transition(
                &self.model,
                self.dt,
                self.last_rudder,
                self.last_throttle,
                x,
            )
        };

        let (x, jac) = jacobian(f, self.state_estimate.into());

        let H = HMatrix::from_fn(|i, j| jac[(physics::motorboat_model::STATE_POSITION_X + i, j)]);

        let S = H * self.covariance_estimate * H.transpose() + R;

        let K = self.covariance_estimate * H.transpose() * S.try_inverse().unwrap();

        self.state_estimate = (x + K * y).into();
        self.covariance_estimate =
            (CovarianceMatrix::identity() - K * H) * self.covariance_estimate;
    }
}

fn h(state: &[Float; physics::motorboat_model::NUM_STATES]) -> SVector<Float, 2> {
    SVector::<Float, 2>::from_row_slice(&[
        state[physics::motorboat_model::STATE_POSITION_X],
        state[physics::motorboat_model::STATE_POSITION_Y],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_kalman_predict() {
        let model = crate::rampage::create_motorboat_model();
        let mut kf = MotorboatDynamicsKalmanFilter::new(0.1, model);

        
        
        kf.predict(0.0, 0.0);

        // assert that the state estimate is finite numbers
        for x in kf.state_estimate.iter() {
            assert!(x.is_finite());
        }

    }
}
