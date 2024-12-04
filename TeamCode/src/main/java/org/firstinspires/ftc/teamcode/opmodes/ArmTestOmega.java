package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class ArmTestOmega extends LinearOpMode {

    private DcMotor arm = hardwareMap.get(DcMotor.class,"ArmOrwhateverr");

    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            if (gamepad1.right_stick_y>0){
                arm.setPower(1);
            } else if (gamepad1.right_stick_y<0) {
                arm.setPower(-1);
            } else if (gamepad1.right_stick_y==0) {
                arm.setPower(0);
            }
        }
    }
}
