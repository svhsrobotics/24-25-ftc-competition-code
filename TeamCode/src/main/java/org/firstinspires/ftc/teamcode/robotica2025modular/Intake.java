package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Intake: the motor that pulls balls in.
 *
 * FROM: DecodeTestTeleOp lines 36 (field), 67 (setup), 208, 215, 222 and 235 (power).
 *
 * The smallest part, but it still gets its own file: if the intake changes (a second motor,
 * a sensor that counts balls), only this file changes.
 */
public class Intake {

    private DcMotor intakeMotor;

    // Find the motor. Called once by RoboticaRobot.init().
    // (Robotica asked for a DcMotorEx and stored it as a DcMotor. Same motor; DcMotor is
    // all this file needs.)
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
    }

    // -1 to 1. X runs it at 1.
    public void run(double power) {
        intakeMotor.setPower(power);
    }

    // Y, and getting ready to shoot.
    public void stop() {
        intakeMotor.setPower(0);
    }
}
