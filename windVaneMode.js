(function() {
const THROTTLE_P_GAIN = 2.0;
const THROTTLE_I_GAIN = 0.1;
const RUDDER_P_GAIN = 2.0;
const RUDDER_I_GAIN = 0.1;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 200);
const rudderPID = new PIDController(RUDDER_P_GAIN, RUDDER_I_GAIN, 0, 100);

function runWindVaneMode() {
    const dx = anchorTarget.x - boat.x, dy = anchorTarget.y - boat.y, dist = Math.sqrt(dx*dx + dy*dy);
    // rotate dx, dy by boat angle to get relative position
    const relDx = dx * Math.cos(boat.angle) + dy * Math.sin(boat.angle);
    const relDy = -dx * Math.sin(boat.angle) + dy * Math.cos(boat.angle);

    // Update debug display
    document.getElementById('debug-display').style.display = 'block';
    document.getElementById('dx-value').textContent = relDx.toFixed(2);
    document.getElementById('dy-value').textContent = relDy.toFixed(2);
    document.getElementById('dist-value').textContent = dist.toFixed(2);

    const est_wind_dir = Math.atan2(kf.x.get([5]), kf.x.get([4]));
    const targetAngle = est_wind_dir + Math.PI; // Point into wind
    let angleError = Math.atan2(Math.sin(targetAngle - boat.angle), Math.cos(targetAngle - boat.angle));
    let angleErrorDeg = angleError * (180/Math.PI);

    let targetRudderAngle = clamp(relDy, -60, 60);

    let rudderOutput = rudderPID.update(targetRudderAngle, -angleErrorDeg, DT);
    controls.rudder = clamp(rudderOutput, -30, 30); // Clamping rudder output

    let throttleOutput = throttlePID.update(0, -relDx, DT);
    controls.throttle = clamp(throttleOutput, 0, 50);

    
}
window.runWindVaneMode = runWindVaneMode; // Expose to global scope
})();
