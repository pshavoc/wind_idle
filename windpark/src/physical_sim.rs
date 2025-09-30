use nalgebra::{Rotation2, SVector, Vector2};
use num_dual::*;

use crate::rampage::*;

const AIR_DENSITY: f32 = 1.225; // kg/m^3, typical value for air density at sea level
const WATER_DENSITY: f32 = 1000.0; // kg/m^3, typical value for water density
const DRAG_COEFFICIENT: f32 = 0.2; // simplified drag coefficient
const ANGULAR_DRAG_COEFFICIENT: f32 = 300.0; // angular drag coefficient

struct RigidBody2D<T: DualNum<f32> + Copy + nalgebra::RealField> {
    position: Vector2<T>,
    orientation: T, // angle in radians
    velocity: Vector2<T>,
    angular_velocity: T,
    mass: T,
    moment_of_inertia: T,
}

impl<T: DualNum<f32> + Copy + nalgebra::RealField> RigidBody2D<T> {
    
    fn apply_force(&mut self, force: Vector2<T>, dt: T) {
        let acceleration = force / self.mass;
        self.velocity += acceleration * dt;
    }

    fn apply_force_at_point(&mut self, force: Vector2<T>, point: Vector2<T>, dt: T) {
        self.apply_force(force, dt);
        let r = point - self.position;
        let torque = r.perp(&force);
        self.apply_torque(torque, dt);
    }

    fn apply_torque(&mut self, torque: T, dt: T) {
        let angular_acceleration = torque / self.moment_of_inertia;
        self.angular_velocity += angular_acceleration * dt;
    }

    fn update(&mut self, dt: T) {
        self.position += self.velocity * dt;
        self.orientation += self.angular_velocity * dt;
    }

    fn body_to_world(&self, local_point: Vector2<T>) -> Vector2<T> {
        let rotation = Rotation2::new(self.orientation);
        self.position + rotation * local_point
    }
}


fn polar_to_vector2<T: DualNum<f32> + Copy + nalgebra::RealField>(magnitude: T, angle_rad: T) -> Vector2<T> {
    Vector2::new(magnitude * angle_rad.cos(), magnitude * angle_rad.sin())
}

pub fn state_transition<D: DualNum<f32> + Copy + nalgebra::RealField>(input: SVector<D, 11>) -> SVector<D, 8> {

    let dt = &input[0];

    // rudder (in radians)
    let rudder = &input[1];
    let throttle = &input[2];

    let position = Vector2::new(input[3].clone(), input[4].clone());
    let orientation = input[5].clone();
    let angular_velocity = input[6].clone();
    let velocity = Vector2::new(input[7].clone(), input[8].clone());
    

    let mut rigid_body = RigidBody2D {
        position,
        orientation,
        velocity,
        angular_velocity,
        mass: MASS.into(),
        moment_of_inertia: MOMENT_OF_INERTIA.into(),
    };

    let wind_velocity = Vector2::new(input[9].clone(), input[10].clone());

    let motor_force = polar_to_vector2(*throttle * 30.0, orientation - *rudder);
    let motor_mount_point = rigid_body.body_to_world(Vector2::new(MOTOR_MOUNT_X.into(), MOTOR_MOUNT_Y.into()));

    rigid_body.apply_force_at_point(motor_force, motor_mount_point, *dt);

    // drag force is the opposite direction of the velocity vector and proportional to the square of the speed
    let speed = velocity.norm();
    if speed > 0.001.into() {
        let dynamic_pressure: D = speed * speed * 0.5 * D::from(WATER_DENSITY);
        let drag_force = velocity.normalize() * (dynamic_pressure * -DRAG_COEFFICIENT);
        let center_of_drag = rigid_body.body_to_world(Vector2::new(CENTER_OF_WATER_DRAG_X.into(), CENTER_OF_WATER_DRAG_Y.into()));
        rigid_body.apply_force_at_point(drag_force, center_of_drag, *dt);
    }

    let relative_wind_velocity = wind_velocity - velocity;
    let wind_dynamic_pressure = relative_wind_velocity.norm_squared() * 0.5 * AIR_DENSITY;

    let wind_force = relative_wind_velocity.normalize() * (wind_dynamic_pressure);
    let center_of_drag = rigid_body.body_to_world(Vector2::new(WIND_CENTER_FORCE_X.into(), WIND_CENTER_FORCE_Y.into()));
    rigid_body.apply_force_at_point(wind_force, center_of_drag, *dt);

    let angular_drag_torque = -D::one().copysign(rigid_body.angular_velocity) * (rigid_body.angular_velocity * rigid_body.angular_velocity * D::from(ANGULAR_DRAG_COEFFICIENT));
    rigid_body.apply_torque(angular_drag_torque, *dt);

    rigid_body.update(*dt);
    SVector::from([
        rigid_body.position[0],
        rigid_body.position[1],
        rigid_body.orientation,
        rigid_body.angular_velocity,
        rigid_body.velocity[0],
        rigid_body.velocity[1],
        wind_velocity[0],
        wind_velocity[1],
    ])
}

#[cfg(test)]
mod tests {
    use super::*;
    use assert_approx_eq::assert_approx_eq;

    #[test]
    fn test_norm_squared() {
        let v = Vector2::new(3.0, 4.0);
        assert_eq!(v.norm_squared(), 25.0);
    }

    #[test]
    fn test_dot() {
        let v1 = Vector2::new(1.0, 0.5);
        let v2 = Vector2::new(3.0, 4.0);
        assert_eq!(v1.dot(&v2), 3.0 + 0.5 * 4.0);
    }

    #[test]
    fn test_body_to_world() {
        let rb = RigidBody2D {
            position: Vector2::new(1.0, 2.0),
            orientation: std::f32::consts::FRAC_PI_2, // 90 degrees
            velocity: Vector2::new(0.0, 0.0),
            angular_velocity: 0.0,
            mass: 1.0,
            moment_of_inertia: 1.0,
        };
        let local_point = Vector2::new(1.0, 0.0);
        let world_point = rb.body_to_world(local_point);
        assert_approx_eq!(world_point[0], 1.0, 1e-6);
        assert_approx_eq!(world_point[1], 3.0, 1e-6);
    }
}