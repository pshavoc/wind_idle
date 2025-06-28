(function() {
const THROTTLE_P_GAIN = 2.0;
const THROTTLE_I_GAIN = 0.1;
const RUDDER_P_GAIN = 2.0;
const RUDDER_I_GAIN = 0.0;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 100);
const rudderPID = new PIDController(RUDDER_P_GAIN, RUDDER_I_GAIN, 0, 1000);

function steering_to_heading(targetAngle_deg) {

    let boat_angle_deg = degrees(boat.angle);

    let rudderOutput = rudderPID.update(targetAngle_deg, boat_angle_deg, DT);
    controls.rudder = clamp(rudderOutput, -30, 30); // Clamping rudder output
}

function logistic(x) {
    // logistic function to smoothly transition from 0 to 1
    return 1.0 / (1.0 + Math.exp(-x));
}

function degrees(radians) {
    // Convert radians to degrees
    return radians * (180 / Math.PI);
}

function runWindVaneMode() {
    const dx = anchorTarget.x - boat.x, dy = anchorTarget.y - boat.y, dist = Math.sqrt(dx*dx + dy*dy);
    // rotate dx, dy by boat angle to get relative position
    const relDx = dx * Math.cos(boat.angle) + dy * Math.sin(boat.angle);
    const relDy = -dx * Math.sin(boat.angle) + dy * Math.cos(boat.angle);

    const boat_vy = boat.gps_vx * Math.cos(boat.angle) + boat.gps_vy * Math.sin(boat.angle);

    // Update debug display
    document.getElementById('debug-display').style.display = 'block';
    document.getElementById('dx-value').textContent = relDx.toFixed(2);
    document.getElementById('dy-value').textContent = relDy.toFixed(2);
    document.getElementById('dist-value').textContent = dist.toFixed(2);

    const LOIT_RADIUS = 25.0;
    const DEFAULT_SPEED = 3.0;
    const LOIT_SPEED_GAIN = 0.5;
    const LOIT_ANGLE_GAIN = 0.75;

    // Point into the wind
    let into_wind_angle_rad = Math.atan2(-kf.x.get([5]), -kf.x.get([4]));
    const is_pointing_into_wind = Math.abs(into_wind_angle_rad - boat.angle) < Math.PI / 2;
    // modify the target angle to 'lean' towards the destination
    into_wind_angle_rad += clamp(relDy * LOIT_ANGLE_GAIN, -Math.PI / 10, Math.PI / 10);

    // todo: add some wind_angle error

    

    const into_wind_angle_deg = degrees(into_wind_angle_rad);
    const bearing_to_deg = degrees(Math.atan2(dy, dx));
    const into_wind_speed = Math.min(relDx * LOIT_SPEED_GAIN, DEFAULT_SPEED) * (is_pointing_into_wind ? 1 : -1);
    const targetAngle = Math.atan2(dy, dx);

    const angle_lerp = logistic(LOIT_RADIUS - dist);
    desired_yaw = angle_lerp * into_wind_angle_deg + (1.0 - angle_lerp) * bearing_to_deg;
    desired_speed = angle_lerp * into_wind_speed + (1.0 - angle_lerp) * DEFAULT_SPEED;

    steering_to_heading(desired_yaw);

    let throttleOutput = throttlePID.update(desired_speed, boat_vy, DT);
    controls.throttle = clamp(throttleOutput, 0, 100);

    
}
window.runWindVaneMode = runWindVaneMode; // Expose to global scope
})();
