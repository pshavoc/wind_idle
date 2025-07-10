(function() {
const THROTTLE_P_GAIN = 100.0;
const THROTTLE_I_GAIN = 0.0;
const RUDDER_P_GAIN = 200.0;
const RUDDER_I_GAIN = 10.0;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 200);
const rudderPID = new PIDController(RUDDER_P_GAIN, RUDDER_I_GAIN, 0, 200);

let counter = 0;

function control(desiredTurnRate, desiredSpeed) {


    // get the boat's forward speed
    const forwardSpeed = Math.cos(boat.angle) * boat.velocity.x + Math.sin(boat.angle) * boat.velocity.y;

    if (counter++ % 100 === 0) {
        console.log(`Desired Turn Rate: ${desiredTurnRate.toFixed(2)} rad/s. Turn Rate: ${boat.angularVelocity.toFixed(2)} rad/s`);
        console.log(`Desired Speed: ${desiredSpeed.toFixed(2)} m/s. Forward Speed: ${forwardSpeed.toFixed(2)} m/s`);    
    }

    

    // Update PID controllers
    const rudderOutput = clamp(rudderPID.update(desiredTurnRate, boat.angularVelocity, DT), -100, 100);
    const throttleOutput = clamp(throttlePID.update(desiredSpeed, forwardSpeed, DT), 0.0, 100.0);
    

    controls.throttle = clamp( Math.sqrt( rudderOutput * rudderOutput + throttleOutput * throttleOutput), 0, 100);
    controls.rudder = clamp (Math.atan2(rudderOutput, throttleOutput) * (180 / Math.PI), -30, 30);
}


// determine the desired turn rate to achieve a heading
function steering_to_heading(targetHeading_deg) {

    // Convert target heading to radians
    const targetHeading_rad = degreesToRadians(targetHeading_deg);

    // Calculate the angle error
    let angleError = targetHeading_rad - boat.angle;

    // Normalize the angle error to be within -PI to PI
    angleError = (angleError + Math.PI) % (2 * Math.PI) - Math.PI;

    // Calculate the desired turn rate
    let desiredTurnRate = angleError / DT; // radians per second
    desiredTurnRate = clamp(desiredTurnRate, -0.349, 0.349); // Limit the turn rate to a maximum of 20 deg/s

    return desiredTurnRate;
}

function normalizeAngle(angle) {
    // Normalize angle to be within 0 to 2 PI

    angle = (angle + 2 * Math.PI) % (2 * Math.PI);

    return angle;
}

function logistic(x) {
    // logistic function to smoothly transition from 0 to 1
    return 1.0 / (1.0 + Math.exp(-x));
}



function runWindVaneMode() {

    

    const dx = anchorTarget.x - boat.pos.x, dy = anchorTarget.y - boat.pos.y, dist = Math.sqrt(dx*dx + dy*dy);

    // // const wind_vec = kalmanFilter.getWindVelocityEstimate();
    const wind_vec = trueWind.clone(); // Use the true wind vector for now
    const wind_angle = wind_vec.angle();
    let into_wind_angle_rad = wind_angle + Math.PI;
    let into_wind_angle_deg = radiansToDegrees(into_wind_angle_rad);
    


    // // rotate dx, dy by wind angle to get relative position
    const relDx = dx * Math.cos(-wind_angle) - dy * Math.sin(-wind_angle);
    const relDy = dx * Math.sin(-wind_angle) + dy * Math.cos(-wind_angle);

    // modify the into_wind_angle_rad based on the relative position
    into_wind_angle_deg += clamp(-relDy * 1.0, -30, 30); // Adjust the angle based on the relative position

    let desiredTurnRate = steering_to_heading(into_wind_angle_deg);

    // modify the desired forward speed based on the distance to the anchor
    let desiredSpeed = clamp(-relDx * 0.1, -2.0, 2.0); // Adjust the speed based on the relative position

    

    // // Update debug display
    document.getElementById('debug-display').style.display = 'block';
    document.getElementById('dx-value').textContent = relDx.toFixed(2);
    document.getElementById('dy-value').textContent = relDy.toFixed(2);
    document.getElementById('dist-value').textContent = dist.toFixed(2);

    // const boat_vy = boat.velocity.x * Math.cos(boat.angle) + boat.velocity.y * Math.sin(boat.angle);

    

    const LOIT_RADIUS = 50.0;
    const DEFAULT_SPEED = 3.0;
    // const LOIT_SPEED_GAIN = 0.5;
    // const LOIT_ANGLE_GAIN = 0.001;

    // // Point into the wind
    
    // const is_pointing_into_wind = Math.abs(into_wind_angle_rad - boat.angle) < Math.PI / 2;
    // // modify the target angle to 'lean' towards the destination
    // into_wind_angle_rad += clamp(relDy * LOIT_ANGLE_GAIN, -Math.PI / 10, Math.PI / 10);

    // // todo: add some wind_angle error

    // const boatDirection = new Vector2D(Math.cos(boat.angle), Math.sin(boat.angle));

    // const bearing_to = Math.atan2(dy, dx);
    // const into_wind_speed = Math.min(-relDx * LOIT_SPEED_GAIN, DEFAULT_SPEED) * (is_pointing_into_wind ? 1 : -1);
    // // const into_wind_speed = 1.0;

    // move the anchor target 25 meters downwind
    let modifed_target = new Vector2D(anchorTarget.x, anchorTarget.y);
    modifed_target.x -= 25 * Math.sin(into_wind_angle_rad);
    modifed_target.y -= 25 * Math.cos(into_wind_angle_rad);
    modifed_target.subtract(boat.pos);
    
    // calc distance from boat to modified target
    const modified_dist = modifed_target.length();

    const wv_turn_rate = steering_to_heading(into_wind_angle_deg);
    const bearing_to = steering_to_heading(radiansToDegrees(Math.atan2(dy, dx)));
    const wv_speed = clamp(-relDx * 0.1, -2.0, 2.0); // Adjust the speed based on the relative position

    const angle_lerp = logistic(LOIT_RADIUS - dist);
    let desired_yaw = angle_lerp * wv_turn_rate + (1.0 - angle_lerp) * bearing_to;
    let desired_speed = angle_lerp * wv_speed + (1.0 - angle_lerp) * DEFAULT_SPEED;


    control(desired_yaw, desired_speed);
    


    

    // let angleError = Math.atan2(Math.sin(desired_yaw - boat.angle), Math.cos(desired_yaw - boat.angle));
    // let desiredTurnRate = clamp(angleError * 2.0, -20.0, 20.0);;

    // let rudderOutput = rudderPID.update(desiredTurnRate, boat.angularVelocity, DT);
    // // if (Math.abs(rudderOutput) > 30.0) {
    //     // desired_speed += Math.abs(rudderOutput) * 0.3;
    // // }
    // controls.rudder = clamp(rudderOutput, -30, 30); // Clamping rudder output

    // let throttleOutput = throttlePID.update(desired_speed, boat.velocity.length(), DT);
    // controls.throttle = clamp(throttleOutput, 0, 100);

    // if (counter++ % 100 === 0) {
    //     console.log(`Boat Angle: ${boat.angle.toFixed(2)} rad, Into Wind Angle: ${into_wind_angle_rad.toFixed(2)} rad, Bearing to Target: ${bearing_to.toFixed(2)} rad`);
    //     console.log(`Boat Velocity: ${boat.velocity.length().toFixed(2)} m/s`);
    //     console.log(`Wind Vane Mode: desired_yaw=${desired_yaw.toFixed(2)}, desired_speed=${desired_speed.toFixed(2)}, relDx=${relDx.toFixed(2)}, relDy=${relDy.toFixed(2)}, dist=${dist.toFixed(2)}`);
    // }

    
}
window.runWindVaneMode = runWindVaneMode; // Expose to global scope
})();
