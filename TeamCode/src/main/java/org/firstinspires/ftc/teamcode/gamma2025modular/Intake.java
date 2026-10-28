package org.firstinspires.ftc.teamcode.gamma2025modular;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * Intake: the motor that pulls balls in (or spits them out).
 *
 * FROM: BobComp lines 36, 66, 99 (setup) and 191 (power).
 *
 * The smallest part, but it still gets its own file: if the intake changes (a second motor,
 * a sensor that counts balls), only this file changes.
 */
public class Intake {

    private DcMotor intakeMotor;

    // Find the motor and set its direction. Called once by BobRobot.init().
    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
    }

    // -1 to 1. Which sign means "in" depends on how the motor is mounted.
    public void run(double power) {
        intakeMotor.setPower(power);
    }
}
