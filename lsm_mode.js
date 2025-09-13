(function() {

const THROTTLE_P_GAIN = 50.0;
const THROTTLE_I_GAIN = 0.0;
const RUDDER_P_GAIN = 50.0;
const RUDDER_I_GAIN = 6.0;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 100);
const rudderPID = new PIDController(RUDDER_P_GAIN, RUDDER_I_GAIN, 0, 1);

let counter = 0;
let gear = 0;
let previousButtonStates = {};

// let gp = null;

function control(desiredTurnRate, desiredSpeed) {

    // get the boat's forward speed
    const forwardSpeed = Math.cos(boat.angle) * boat.velocity.x + Math.sin(boat.angle) * boat.velocity.y;
    // const forwardSpeed = boat.speed;

    if (counter++ % 100 === 0) {
        console.log(`Desired Turn Rate: ${desiredTurnRate.toFixed(2)} rad/s. Turn Rate: ${boat.angularVelocity.toFixed(2)} rad/s`);
        console.log(`Desired Speed: ${desiredSpeed.toFixed(2)} m/s. Forward Speed: ${forwardSpeed.toFixed(2)} m/s`);    
    }

    

    // Update PID controllers
    let rudderOutput = clamp(rudderPID.update(desiredTurnRate, boat.angularVelocity, DT), -100, 100);
    let throttleOutput = clamp(throttlePID.update(desiredSpeed, forwardSpeed, DT), -100.0, 100.0);
    
    const throttleSign = (throttleOutput > 0) ? 1 : -1;

    throttleOutput = clamp( throttleSign * Math.sqrt( rudderOutput * rudderOutput + throttleOutput * throttleOutput), -100, 100);

    // add slew rate limiting
    let diff = throttleOutput - controls.throttle;
    diff = clamp(diff, -5 * DT, 5 * DT);
    controls.throttle = clamp(controls.throttle + diff, -20, 100);

    rudderOutput = clamp ( Math.sign(controls.throttle) * Math.atan2(rudderOutput, controls.throttle) * (180 / Math.PI), -30, 30);
    diff = rudderOutput - controls.rudder;
    diff = clamp(diff, -40 * DT, 40 * DT);
    controls.rudder = controls.rudder + diff;

}

function lsmModeControl() {
    const gamepads = navigator.getGamepads();
    if (!gamepads) return;

    const gp = gamepads[0];
    if (!gp) return;

    if (gp === null) return;

    if (!previousButtonStates[gp.index]) {
        previousButtonStates[gp.index] = Array(gp.buttons.length).fill(false);
    }

    if (gp.buttons[12].pressed && !previousButtonStates[gp.index][12]) {
        gear += 1;
    }
    if (gp.buttons[13].pressed && !previousButtonStates[gp.index][13]) {
        gear -= 1;
    }

    previousButtonStates[gp.index] = gp.buttons.map(b => b.pressed);

    
    gear = clamp(gear, -5, 10);

    let desiredSpeed = gear * gp.buttons[6].value;

    
    control(gp.axes[0] * 0.5, desiredSpeed);

    // console.log(`gp.axes[0]: ${gp.axes[0]}`);
    // console.log(`gp.axes[1]: ${gp.axes[1]}`);
}

// var haveEvents = 'GamepadEvent' in window;
// var haveWebkitEvents = 'WebKitGamepadEvent' in window;

// function scangamepads() {
//   var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads() : []);
//   for (var i = 0; i < gamepads.length; i++) {
//     if (gamepads[i] && (gamepads[i].index in controllers)) {
//       controllers[gamepads[i].index] = gamepads[i];
//     }
//   }
// }

// function connecthandler(e) {
//   gp = e.gamepad;
// }

// function disconnecthandler(e) {
//   gp = null;
// }

// if (haveEvents) {
//   window.addEventListener("gamepadconnected", connecthandler);
//   window.addEventListener("gamepaddisconnected", disconnecthandler);
// } else if (haveWebkitEvents) {
//   window.addEventListener("webkitgamepadconnected", connecthandler);
//   window.addEventListener("webkitgamepaddisconnected", disconnecthandler);
// } else {
//   setInterval(scangamepads, 500);
// }

window.lsmModeControl = lsmModeControl; // Expose to global scope
})();

