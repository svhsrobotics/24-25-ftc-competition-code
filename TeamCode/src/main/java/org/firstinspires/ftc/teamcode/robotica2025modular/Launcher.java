package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Launcher: everything about shooting. Two flywheels (a small one and a big one, spun
 * together) AND the two trigger servos. "Shooting" is one job, so it lives in one file,
 * like Gamma's pushers.
 *
 * WHAT: Remembers an adjustable speed, spins both flywheels (the big one at 0.75 of the small
 *       one), stops them, and moves the triggers.
 * FROM: DecodeTestTeleOp lines 33 to 38 and 50 (fields), 65 to 66, 68, 71, 79 to 80, 85 to 86
 *       and 92 to 93 (setup), 99 to 100 (triggers at INIT), 204 to 220 (spin and stop),
 *       223 to 226 (X), 228 to 233 (fine-tune), 237 to 244 (triggers).
 */
public class Launcher {

    // The big flywheel's radius is about 33% larger, so it spins at 0.75 of the small one's
    // speed to give the ball the same surface speed (comment on line 206). 1 / 1.33 = 0.75.
    private static final double BIG_RATIO = 0.75;

    // Trigger servo positions (0 to 1 of the servo's travel), found by testing on Robotica.
    // The names "rest" and "release" are guesses: the team knows what each position does.
    private static final double LEFT_REST = 0.47;
    private static final double RIGHT_REST = 0.515;
    private static final double LEFT_RELEASE = 0.52;
    private static final double RIGHT_RELEASE = 0.449;

    private DcMotorEx small;   // Robotica: smallLaunchMotor. DcMotorEx because it has setVelocity
    private DcMotorEx big;     // Robotica: bigLaunchMotor
    private Servo leftTrigger;
    private Servo rightTrigger;

    private double targetSpeed = 0.46;   // Robotica: smallLaunchSpeed

    // Find the motors and servos and set them up. Called once by RoboticaRobot.init().
    public void init(HardwareMap hardwareMap) {
        small = hardwareMap.get(DcMotorEx.class, "smallLaunchMotor");
        big = hardwareMap.get(DcMotorEx.class, "bigLaunchMotor");
        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        small.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        big.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Lines 85 to 86 turn the encoders off. See the NOTE on turnOn(speed): the first
        // setVelocity call turns them back on.
        small.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        big.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        big.setDirection(DcMotorSimple.Direction.REVERSE);
        small.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ---- flywheels ---------------------------------------------------------------------

    // Spin up at the adjustable speed. Not used by RoboticaTeleOp (it passes the speed to
    // RoboticaRobot.getReadyToShoot), but kept in the approved design for other OpModes.
    public void turnOn() {
        turnOn(targetSpeed);
    }

    /*
     * Spin up at the given speed, big wheel at 0.75 of it. Doesn't change the adjustable speed.
     *
     * NOTE (same as Robotica): setVelocity takes a speed in encoder TICKS PER SECOND, not a
     * power from 0 to 1. In the SDK 11 code (checked 2026-10-26), setVelocity:
     *   1. switches the motor to RUN_USING_ENCODER if it isn't already (undoing lines 85 to 86),
     *   2. rounds the speed to a whole number of ticks per second.
     * So 0.46 becomes 0 and 0.65 becomes 1: the flywheels are held almost still. Verify on
     * the robot. Then pick one: setPower with power levels (0 to 1), or setVelocity with
     * real speeds in ticks per second (Gamma used 750 to 900).
     */
    public void turnOn(double speed) {
        small.setVelocity(speed);
        big.setVelocity(BIG_RATIO * speed);
    }

    public void turnOff() {
        big.setVelocity(0);
        small.setVelocity(0);
    }

    /*
     * Fine-tune the adjustable speed (D-pad left plus B or X).
     * NOTE (same as Robotica): 0.00001 per loop pass while held. At about 50 passes a second,
     * changing it by 0.01 takes about 20 seconds of holding.
     */
    public void changeTargetSpeed(double amount) {
        targetSpeed = targetSpeed + amount;
    }

    public double getTargetSpeed() {
        return targetSpeed;
    }

    // ---- triggers ----------------------------------------------------------------------

    // Left bumper.
    public void release() {
        leftTrigger.setPosition(LEFT_RELEASE);
        rightTrigger.setPosition(RIGHT_RELEASE);
    }

    // Right bumper, X (see RoboticaRobot.startIntaking), and INIT.
    public void rest() {
        leftTrigger.setPosition(LEFT_REST);
        rightTrigger.setPosition(RIGHT_REST);
    }
}
