package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class RaiseyUppyThing7 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        waitForStart();

        int apressed = 0;

        arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        while (opModeIsActive()){
                if (gamepad1.a) {
                    arm.setPower(0.4);
                }
                else {
                    arm.setPower(0);
                }
        }
    }
}
