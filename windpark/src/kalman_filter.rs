
use num_dual::jacobian;

type CovarianceMatrix = nalgebra::SMatrix<Float, {physics::motorboat_model::NUM_STATES}, {physics::motorboat_model::NUM_STATES}>;

use crate::{Float, physics};
pub struct MotorboatDynamicsKalmanFilter {
    model: physics::motorboat_model::MotorboatModel,
    state_estimate: [Float; physics::motorboat_model::NUM_STATES],
    covariance_estimate: CovarianceMatrix,
    last_rudder: Float,
    last_throttle: Float,
    last_dt: Float,

}


impl MotorboatDynamicsKalmanFilter {

    pub fn new() -> Self {
        Self {
            model: todo!(),
            state_estimate: [0.0; physics::motorboat_model::NUM_STATES],
            covariance_estimate: CovarianceMatrix::identity(),
            last_rudder: 0.0,
            last_throttle: 0.0,
            last_dt: 0.0,
        }
    }


    pub fn predict(&mut self, rudder: Float, throttle: Float, dt: Float) {

        self.last_rudder = rudder;
        self.last_throttle = throttle;
        self.last_dt = dt;

        let f = |x| {
            physics::motorboat_model::state_transition(
                &self.model,
                dt,
                rudder,
                throttle,
                x,
            )
        };

        let (x, jac) = jacobian(f, self.state_estimate.into());

        self.state_estimate = x.into();

        self.covariance_estimate = jac * self.covariance_estimate * jac.transpose();



        // let x_hat = physics::motorboat_model::state_transition(
        //     &self.model,
        //     dt,
        //     rudder,
        //     throttle,
        //     self.state_estimate.into(),
        // );

        // self.state_estimate = x_hat.into();

    }

    pub fn update_gps(&mut self, gps_position: [Float; 2]) {

        // let f = |x| {
        //     physics::motorboat_model::state_transition(
        //         &self.model,
        //         dt,
        //         rudder,
        //         throttle,
        //         x,
        //     )
        // };

        // let (j, x) = jacobian(f, self.state_estimate.into());

    }
}