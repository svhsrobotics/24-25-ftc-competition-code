package org.firstinspires.ftc.teamcode.gamma2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * Launcher: everything about shooting a ball. The two flywheels, AND the two pusher servos
 * that push a ball into them. "Shooting" is one job, so it lives in one file.
 *
 * WHAT: Remembers a target speed and whether the flywheels are on, spins them at that speed,
 *       says whether they've reached it, picks a speed from the goal's distance, and pushes.
 * FROM: BobComp lines 34 to 35, 37 to 38, 40, 47 (fields), 64 to 65, 67 to 68, 95 to 98 (setup),
 *       118 to 122 (at-speed check), 137 to 160 (speed changes), 164 to 170 (pushers),
 *       172 to 188 (on/off), 273 to 281 (distance table).
 *
 * Speeds are in encoder ticks per second, not power. setVelocity() uses each motor's encoder
 * to HOLD that speed as the battery drains, so shots stay the same all match. setPower() would
 * just push harder or softer, and the speed would sag with the battery.
 */
public class Launcher {

    // The "at speed" band: a measured speed jitters and is never exactly the target,
    // so "close enough" means within 50 ticks/s either way.
    private static final double AT_SPEED_TOLERANCE = 50;

    // The speed presets on the bumpers (ticks per second).
    public static final double PRESET_LOW = 750;
    public static final double PRESET_HIGH = 900;

    // Pusher servo positions (0 to 1 of the servo's travel), found by testing on Bob.
    // The two servos face each other, so their numbers mirror.
    private static final double LEFT_REST = 0.86;
    private static final double RIGHT_REST = 0.3;
    private static final double LEFT_PUSH = 0.14;
    private static final double RIGHT_PUSH = 0.84;

    private DcMotorEx leftShoot;   // DcMotorEx (not DcMotor) because it has setVelocity/getVelocity
    private DcMotorEx rightShoot;
    private Servo leftPush;
    private Servo rightPush;

    private double targetSpeed = 0;    // BobComp called this "shoot". Starts at 0 (Java's default)
    private boolean isOn = false;      // BobComp called this "shouldShoot"

    // Find the motors and servos and set their directions. Called once by BobRobot.init().
    public void init(HardwareMap hardwareMap) {
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");
        leftPush = hardwareMap.get(Servo.class, "leftPush");
        rightPush = hardwareMap.get(Servo.class, "rightPush");

        // The two flywheel motors face each other, so one is reversed to make both wheels throw
        // the ball the same way. Depends on how the launcher is built: test each motor slowly.
        leftShoot.setDirection(DcMotor.Direction.FORWARD);
        rightShoot.setDirection(DcMotor.Direction.REVERSE);

        // setDirection on a servo flips what its positions mean (0 becomes 1). Bob's autos set
        // rightPush to REVERSE, so the same numbers meant different spots there. Keeping the
        // setup in this one file is what prevents that kind of drift.
        leftPush.setDirection(Servo.Direction.FORWARD);
        rightPush.setDirection(Servo.Direction.FORWARD);
    }

    // ---- flywheel speed ----------------------------------------------------------------

    public double getTargetSpeed() {
        return targetSpeed;
    }

    public void setTargetSpeed(double ticksPerSecond) {
        targetSpeed = ticksPerSecond;
    }

    // Add to (or subtract from) the target speed. Used by the D-pad.
    // NOTE (same as Bob): there's no lower limit, so the target can go below 0 (spins backward).
    public void changeTargetSpeed(double amount) {
        targetSpeed += amount;
    }

    public void turnOn() {
        isOn = true;
    }

    public void turnOff() {
        isOn = false;
    }

    // Call every loop pass. Sends the current target to the motors (or 0 if off).
    // Running every pass means a speed change takes effect on the very next pass.
    public void update() {
        if (isOn) {
            leftShoot.setVelocity(targetSpeed);
            rightShoot.setVelocity(targetSpeed);
        } else {
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        }
    }

    // True when BOTH wheels are within the band around the target, and the target isn't 0.
    // Why "target isn't 0": a stopped launcher (speed 0, target 0) would otherwise count as
    // "at speed" and the screen would say it's ready when it isn't.
    public boolean isAtSpeed() {
        return leftShoot.getVelocity() < targetSpeed + AT_SPEED_TOLERANCE
                && leftShoot.getVelocity() > targetSpeed - AT_SPEED_TOLERANCE
                && rightShoot.getVelocity() < targetSpeed + AT_SPEED_TOLERANCE
                && rightShoot.getVelocity() > targetSpeed - AT_SPEED_TOLERANCE
                && targetSpeed != 0;
    }

    /*
     * Pick the target speed from the distance to the goal (centimeters), measured on Bob.
     * The checks go from nearest to farthest. "else if" only runs when every check above it
     * was false, so each line covers one band: under 170, 170 to 200, 200 to 210, 210 to 217.
     * NOTE (same as Bob): at 217 cm or farther nothing matches, so the target stays where it
     * was. Bob's comment said far shots were inconsistent, so the driver set those by hand.
     */
    public void setSpeedForDistance(double distanceCm) {
        if (distanceCm < 170) {
            targetSpeed = 750;
        } else if (distanceCm < 200) {
            targetSpeed = 760;
        } else if (distanceCm < 210) {
            targetSpeed = 775;
        } else if (distanceCm < 217) {
            targetSpeed = 790;
        }
    }

    // ---- pushers -----------------------------------------------------------------------

    // Push a ball into the flywheels.
    public void push() {
        leftPush.setPosition(LEFT_PUSH);
        rightPush.setPosition(RIGHT_PUSH);
    }

    // Pull the pushers back out of the way.
    public void rest() {
        leftPush.setPosition(LEFT_REST);
        rightPush.setPosition(RIGHT_REST);
    }
}
