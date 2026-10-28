package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Drivetrain: the four mecanum wheels.
 *
 * WHAT: Turns "forward / strafe / turn" into four wheel powers.
 * FROM: DecodeTestTeleOp lines 29 to 32 (fields), 61 to 64, 74 to 78 and 90 to 91 (setup),
 *       193 to 200 (driving).
 *
 * Only this file knows there are four wheels. Everything else just says
 * "drive this way" or "turn this much".
 */
public class Drivetrain {

    private DcMotor leftFront;    // Robotica: leftFrontMotor
    private DcMotor leftBack;
    private DcMotor rightFront;
    private DcMotor rightBack;

    // Find the motors, make them brake, set their directions. Called once by RoboticaRobot.init().
    public void init(HardwareMap hardwareMap) {
        // Names in quotes must match the robot's configuration exactly (capitals count).
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        // BRAKE: at power 0 the motor resists turning, so the robot stops quickly when the
        // sticks are let go. (The other choice, FLOAT, lets it coast.)
        leftFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // The left motors are mounted as mirror images of the right ones. Reversing the left
        // side means "+ power = robot forward" for all four wheels, which keeps the math simple.
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    /*
     * Drive with three separate requests, each from -1 to 1:
     *   forward: + = forward, - = backward
     *   strafe:  + = slide right, - = slide left
     *   turn:    + = turn right (clockwise), - = turn left
     *
     * Each wheel's power = forward + strafe + turn, with a + or - for its corner
     * (Robotica's math, unchanged; Robotica called these y, x and rx):
     *
     *                  forward   strafe   turn
     *     leftFront       +         +       +
     *     leftBack        +         -       +
     *     rightFront      +         -       -
     *     rightBack       +         +       -
     *
     * Why those signs: forward = every wheel rolls forward. Turn = left side forward, right
     * side backward, like a tank. Strafe = the mecanum rollers sit at 45 degrees; when one
     * diagonal pair rolls forward and the other rolls backward, the sideways pushes add up
     * and the forward pushes cancel, so the robot slides.
     *
     * NOTE (same as Robotica): no cap. If the sum is over 1, the motor just runs at 1, so
     * combined moves come out a little different than the math asks for.
     */
    public void drive(double forward, double strafe, double turn) {
        leftFront.setPower(forward + strafe + turn);
        leftBack.setPower(forward - strafe + turn);
        rightFront.setPower(forward - strafe - turn);
        rightBack.setPower(forward + strafe - turn);
    }

    // Spin in place: + = left wheels forward, right wheels backward (turn right).
    // That's the "turn" column above on its own. Used to center on an AprilTag.
    public void turnInPlace(double power) {
        drive(0, 0, power);
    }
}
