

/// Rampage is 16 feet long, converted to meters.
pub const LENGTH: f32 = 16.0 * 0.3048;

/// Rampage is about 6.5 feet wide, converted to meters.
pub const WIDTH: f32 = 6.5 * 0.3048;

/// Rampage is about 600lbs, converted to kg.
pub const MASS: f32 = 800.0 * 0.453592;

/// Rampage moment of inertia, approximated as a rectangle.
pub const MOMENT_OF_INERTIA: f32 = MASS * (LENGTH * WIDTH * WIDTH * WIDTH) / 12.0;

pub const MOTOR_MOUNT_X: f32 = -3.0 * 0.3048;
pub const MOTOR_MOUNT_Y: f32 = 0.0;

pub const CENTER_OF_WATER_DRAG_X: f32 = -1.0 * 0.3048; // 1 feet from center of mass
pub const CENTER_OF_WATER_DRAG_Y: f32 = 0.0;

pub const WIND_CENTER_FORCE_X: f32 = 2.0 * 0.3048;
pub const WIND_CENTER_FORCE_Y: f32 = 0.0;
