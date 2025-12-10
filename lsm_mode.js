import { ekf, controls } from './app.js';
import { PIDController } from './PIDController.js';
import { clamp, degreesToRadians } from './utils.js';

const THROTTLE_P_GAIN = 10.0;
const THROTTLE_I_GAIN = 10.0;
const RUDDER_P_GAIN = 10.0;
const RUDDER_I_GAIN = 20.0;

const throttlePID = new PIDController(THROTTLE_P_GAIN, THROTTLE_I_GAIN, 0, 100);
const rudderPID = new PIDController(RUDDER_P_GAIN, RUDDER_I_GAIN, 0, 1);

let last_time = 0.0;

let gear = 0;
let previousButtonStates = {};

function getControllerInput() {

    let speed = 0.0;
    let turnRate = 0.0;

    const gamepads = navigator.getGamepads();
    if (!gamepads) return { speed, turnRate };

    const gp = gamepads[0];
    if (!gp) return { speed, turnRate };

    if (gp === null) return { speed, turnRate };

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

    speed = gear / 10.0 * gp.buttons[6].value;
    turnRate = gp.axes[0];

    return { speed, turnRate };

}


function manualModeControl() {
    const { speed, turnRate } = getControllerInput();
    controls.throttle = clamp(100.0 * speed, -100, 100);
    controls.rudder = clamp(15.0 * turnRate / 0.349, -15, 15);
}

function acroModeControl() {

    const now = performance.now();
    const dt = (now - last_time) / 1000.0;
    last_time = now;

    const { speed, turnRate } = getControllerInput();

    // max speed of boat is 8 m/s
    const desiredSpeed = speed * 8.0;

    const velocity = ekf.velocity();
    const vx = velocity[0];
    const vy = velocity[1];
    const angle = ekf.theta();
    const angularVelocity = ekf.angular_velocity();
    const forwardSpeed = Math.cos(angle) * vx + Math.sin(angle) * vy;

    // max turn rate of boat is 15 degrees per second
    const desiredTurnRate = degreesToRadians(turnRate * 20.0);

    // console.log(`desiredSpeed: ${desiredSpeed.toFixed(2)}, forwardSpeed: ${forwardSpeed.toFixed(2)}`);
    // console.log(`desiredTurnRate: ${desiredTurnRate.toFixed(2)}, angularVelocity: ${angularVelocity.toFixed(2)}`);

    // Update PID controllers
    controls.rudder = clamp(rudderPID.update(desiredTurnRate, angularVelocity, dt), -15, 15);
    controls.throttle = clamp(throttlePID.update(desiredSpeed, forwardSpeed, dt), -100.0, 100.0);

}

export { manualModeControl, acroModeControl };

