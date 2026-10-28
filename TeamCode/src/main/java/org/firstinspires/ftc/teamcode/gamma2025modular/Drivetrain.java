package org.firstinspires.ftc.teamcode.gamma2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Drivetrain: the four mecanum wheels.
 *
 * WHAT: Turns "forward / strafe / turn" into four wheel powers.
 * FROM: BobComp lines 30 to 33, 60 to 63, 91 to 94 (setup), 251 to 254 (driving),
 *       316 to 319 (turning in place for auto-aim).
 *
 * Only this file knows there are four wheels. Everything else just says
 * "drive this way" or "turn this much".
 */
public class Drivetrain {

    // Bob drove at most 85% power, for control.
    private static final double MAX_DRIVE_POWER = 0.85;

    private DcMotor leftFront;
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Find the motors and set their directions. Called once by BobRobot.init().
    public void init(HardwareMap hardwareMap) {
        // Names in quotes must match the robot's configuration exactly (capitals count).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // The left motors are mounted as mirror images of the right ones. Reversing the left
        // side means "+ power = robot forward" for all four wheels, which keeps the math simple.
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    /*
     * Drive with three separate requests, each from -1 to 1:
     *   forward: + = forward, - = backward
     *   strafe:  + = slide right, - = slide left
     *   turn:    + = turn right (clockwise), - = turn left
     *
     * Each wheel's power = forward + strafe + turn, with a + or - for its corner:
     *
     *                  forward   strafe   turn
     *     leftFront       +         +       +
     *     leftBack        +         -       +
     *     rightFront      +         -       -
     *     rightBack       +         +       -
     *
     * Why those signs:
     *   forward: every wheel rolls forward.
     *   turn:    left side forward, right side backward = spin in place, like a tank.
     *   strafe:  mecanum rollers sit at 45 degrees. When one diagonal pair (leftFront and
     *            rightBack) rolls forward and the other pair rolls backward, their sideways
     *            pushes add up and their forward/backward pushes cancel out. The robot slides.
     *            (This only works with mecanum wheels mounted the usual way.)
     *
     * Try it: full forward (1, 0, 0) gives all four wheels +0.85.
     *         full turn right (0, 0, 1) gives left wheels +0.85, right wheels -0.85.
     *
     * NOTE (same as Bob): if the sum is over 1 (say forward 1 and turn 0.3 = 1.3), the motor
     * just runs at 1, so combined moves come out a little different than the math asks for.
     * The usual fix is to divide all four by the largest one when it's over 1. Not done here,
     * to match Bob exactly.
     */
    public void drive(double forward, double strafe, double turn) {
        leftFront.setPower(MAX_DRIVE_POWER * (forward + strafe + turn));
        leftBack.setPower(MAX_DRIVE_POWER * (forward - strafe + turn));
        rightFront.setPower(MAX_DRIVE_POWER * (forward - strafe - turn));
        rightBack.setPower(MAX_DRIVE_POWER * (forward + strafe - turn));
    }

    /*
     * Spin in place at the given power, used by auto-aim.
     * + = left wheels forward and right wheels backward. That's the "turn" column above,
     * without the 85% cap (BobComp's aim code didn't use the cap either).
     */
    public void turnInPlace(double turnPower) {
        leftFront.setPower(turnPower);
        leftBack.setPower(turnPower);
        rightFront.setPower(-turnPower);
        rightBack.setPower(-turnPower);
    }
}
