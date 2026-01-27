use autodiff_emb::*;
use nalgebra::{SVector, SVectorView, Vector1, Vector2, matrix};

use wasm_bindgen::prelude::*;

const KF_NUM_STATES: usize = physics::motorboat_model::NUM_STATES + 1;
const STATE_COMPASS_OFFSET: usize = physics::motorboat_model::NUM_STATES;

type CovarianceMatrix = nalgebra::SMatrix<Float, KF_NUM_STATES, KF_NUM_STATES>;

use crate::{
    Float,
    physics::{
        self,
        motorboat_model::{STATE_ORIENTATION, STATE_POSITION_X, STATE_POSITION_Y},
    }, wrap_angle,
};

#[wasm_bindgen]
pub struct MotorboatDynamicsKalmanFilter {
    model: physics::motorboat_model::MotorboatModel,
    state_estimate: SVector<Float, { KF_NUM_STATES }>,
    covariance_estimate: CovarianceMatrix,
    process_noise: CovarianceMatrix,
    dt: Float,
}

#[wasm_bindgen]
impl MotorboatDynamicsKalmanFilter {
    fn new(dt: Float, model: physics::motorboat_model::MotorboatModel) -> Self {
        let process_noise = {
            let mut q = CovarianceMatrix::identity() * 0.001;
            q[(STATE_POSITION_X, STATE_POSITION_X)] = 0.001;
            q[(STATE_POSITION_Y, STATE_POSITION_Y)] = 0.001;
            q[(STATE_ORIENTATION, STATE_ORIENTATION)] = 0.01;
            q[(STATE_COMPASS_OFFSET, STATE_COMPASS_OFFSET)] = 0.0;
            q
        };
        Self {
            model,
            state_estimate: SVector::zeros(),
            covariance_estimate: CovarianceMatrix::identity(),
            process_noise: process_noise,
            dt,
        }
    }

    #[wasm_bindgen(constructor)]
    pub fn create() -> Self {
        let mut model = crate::rampage::create_motorboat_model();

        model.mass *= 1.3;
        model.moment_of_inertia *= 0.8;
        model.water_drag_coefficient *= 1.3;
        model.wind_drag_coefficient *= 0.8;
        model.angular_drag_coefficient *= 1.2;
        model.motor_scaler *= 1.2;

        Self::new(0.1, model)
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
    pub fn wind_velocity(&self) -> Vec<Float> {
        vec![self.state_estimate[6], self.state_estimate[7]]
    }

    #[wasm_bindgen]
    pub fn compass_offset(&self) -> Float {
        self.state_estimate[STATE_COMPASS_OFFSET]
    }

    #[wasm_bindgen]
    pub fn predict(&mut self, rudder: Float, throttle: Float) {
        let f = |x: &nalgebra::SVector<Dual32, { physics::motorboat_model::NUM_STATES }>| {
            physics::motorboat_model::state_transition(&self.model, self.dt, rudder, throttle, x)
        };

        // let (x, jac) = num_dual::jacobian(f, self.state_estimate.into());

        // let x: nalgebra::SVector<Float, { physics::motorboat_model::NUM_STATES }> =
        //     nalgebra::SVector::from_row_slice(
        //         &self.state_estimate[..physics::motorboat_model::NUM_STATES],
        //     );

        let x = self
            .state_estimate
            .fixed_rows::<{ physics::motorboat_model::NUM_STATES }>(0);

        let jac = autodiff_emb::jacobian(f, &x);

        let x_hat =
            physics::motorboat_model::state_transition(&self.model, self.dt, rudder, throttle, &x);

        self.state_estimate
            .fixed_rows_mut::<{ physics::motorboat_model::NUM_STATES }>(0)
            .copy_from(&x_hat);

        // Normalize the orientation angle to [-pi, pi] to prevent floating-point numbers from growing indefinitely.
        self.state_estimate[STATE_ORIENTATION] = wrap_angle(self.state_estimate[STATE_ORIENTATION]);

        // expand the jacobian to include the compass offset state
        let mut jac_expanded = CovarianceMatrix::identity();
        jac_expanded
            .fixed_view_mut::<{ physics::motorboat_model::NUM_STATES }, { physics::motorboat_model::NUM_STATES }>(0, 0)
            .copy_from(&jac);

        self.covariance_estimate =
            jac_expanded * self.covariance_estimate * jac_expanded.transpose() + self.process_noise;
    }

    #[allow(non_snake_case)]
    #[wasm_bindgen]
    pub fn update_gps(
        &mut self,
        gps_position_x: Float,
        gps_position_y: Float,
        gps_position_variance: Float,
    ) {
        let z = Vector2::new(gps_position_x, gps_position_y);
        let R = nalgebra::Matrix2::<Float>::identity() * gps_position_variance.abs();

        let h = self.state_estimate.fixed_rows::<2>(STATE_POSITION_X);

        let y = z - h;

        let H = matrix![
            1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
            0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0
        ];

        let S = H * self.covariance_estimate * H.transpose() + R;

        let K = self.covariance_estimate * H.transpose() * S.try_inverse().unwrap();

        let x = self.state_estimate;
        self.state_estimate = (x + K * y).into();
        
        // Normalize the orientation angle to [-pi, pi] to prevent floating-point numbers from growing indefinitely.
        self.state_estimate[STATE_ORIENTATION] = wrap_angle(self.state_estimate[STATE_ORIENTATION]);

        self.covariance_estimate =
            (CovarianceMatrix::identity() - K * H) * self.covariance_estimate;
    }

    #[allow(non_snake_case)]
    #[wasm_bindgen]
    pub fn update_compass(&mut self, compass_heading: Float, compass_heading_variance: Float) {
        let R = nalgebra::Matrix1::<Float>::identity() * compass_heading_variance.abs();

        let z_pred = self.state_estimate[STATE_ORIENTATION] + self.state_estimate[STATE_COMPASS_OFFSET];

        // Normalize angle difference to [-pi, pi] to avoid discontinuities
        let y = Vector1::new(wrap_angle(compass_heading - z_pred));

        let H = matrix![
            0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;
        ];

        let S = H * self.covariance_estimate * H.transpose() + R;

        let K = self.covariance_estimate * H.transpose() * S.try_inverse().unwrap();

        let x = self.state_estimate;
        self.state_estimate = (x + K * y).into();

        // Normalize the orientation angle to [-pi, pi] to prevent floating-point numbers from growing indefinitely.
        self.state_estimate[STATE_ORIENTATION] = wrap_angle(self.state_estimate[STATE_ORIENTATION]);

        self.covariance_estimate =
            (CovarianceMatrix::identity() - K * H) * self.covariance_estimate;
        
        
    }
}

#[cfg(test)]
mod tests {
    use crate::physics::motorboat_model::{STATE_POSITION_X, STATE_POSITION_Y, STATE_VELOCITY_X};

    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_kalman_predict() {
        let model = crate::rampage::create_motorboat_model();
        let mut kf = MotorboatDynamicsKalmanFilter::new(0.1, model);

        // kf.state_estimate[STATE_VELOCITY_X] = 3.0;

        let mut position = Vector2::new(0.0, 0.0);
        let velocity = Vector2::new(1.0, 0.0);

        for _ in 0..30 {
            for i in 0..10 {
                kf.predict(0.0, 0.0);

                println!("state estimate {} {:?}", i, kf.state_estimate);
            }

            position += velocity * 1.0;
            kf.update_gps(position.x, position.y, 1.0);

            println!("expected position: {:?}", position);
            println!("state estimate: {:?}", kf.state_estimate);
        }

        assert_approx_eq!(kf.state_estimate[STATE_POSITION_X], 3.0, 1e-2);
        assert_approx_eq!(kf.state_estimate[STATE_POSITION_Y], 0.0);
    }

    #[test]
    fn test_kalman_update_compass() {
        let model = crate::rampage::create_motorboat_model();
        let mut kf = MotorboatDynamicsKalmanFilter::new(0.1, model);

        kf.predict(0.0, 0.0);
        kf.predict(0.0, 0.0);
        kf.predict(0.0, 0.0);

        kf.update_compass(0.1, 0.01);

        // assert that the state estimate is finite numbers
        for x in kf.state_estimate.iter() {
            assert!(x.is_finite());
        }

        kf.predict(0.0, 0.0);

        // assert that the state estimate is finite numbers
        for x in kf.state_estimate.iter() {
            assert!(x.is_finite());
        }
    }
}
