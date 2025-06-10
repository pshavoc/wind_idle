const AP_MAX_THROTTLE = 40;
const THROTTLE_P_GAIN = 0.5;
const AP_KI_GAIN = 0.05;
const AP_MAX_INTEGRAL_ANCHOR = 400;
const AP_STOPPING_DISTANCE = 5;
const RUDDER_P_GAIN = 0.8;

// Initialize PID controllers
const throttlePID = new PIDController(THROTTLE_P_GAIN, AP_KI_GAIN, 0, AP_MAX_INTEGRAL_ANCHOR);
const rudderPID = new PIDController(RUDDER_P_GAIN, 0, 0, null);

function runAnchorMode() {
    const dx = anchorTarget.x - boat.x, dy = anchorTarget.y - boat.y, dist = Math.sqrt(dx*dx + dy*dy);
    const targetAngle = Math.atan2(dy, dx);
    let angleError = Math.atan2(Math.sin(targetAngle - boat.angle), Math.cos(targetAngle - boat.angle));
    
    // Update rudder PID
    // Convert angleError to degrees for the PID controller as it was originally scaled by (180/Math.PI)
    let rudderOutput = rudderPID.update(0, -(angleError * (180/Math.PI)), DT);
    controls.rudder = clamp(rudderOutput, -30, 30); // Clamping rudder output


    if(Math.abs(angleError) < (Math.PI/1.5)) { // If pointing somewhat towards target
        // Update throttle PID
        // The target for the throttle PID is a distance of 0.
        let throttleOutput = throttlePID.update(0, -dist, DT); // Pass -dist because PID tries to bring error to 0. target - (-dist) = dist.
        controls.throttle = clamp(throttleOutput, 0, AP_MAX_THROTTLE); // Clamping throttle output
    } else {
        controls.throttle = 5; // Fixed throttle for turning, maintain heading
        throttlePID.reset(); // Reset throttle PID when not actively seeking target
    }
}
