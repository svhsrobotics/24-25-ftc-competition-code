package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class ClawTestOmega extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Double power = 0.3;
        DcMotor Claw = hardwareMap.get(DcMotor.class,"claw");
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_bumper) {
                Claw.setPower(power);
            } else if (gamepad1.right_trigger>0.1) {
                Claw.setPower(-power);
            }
            else {
               Claw.setPower(0);
            }
        }
    }
}
