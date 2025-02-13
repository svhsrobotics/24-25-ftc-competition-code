package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

//@TeleOp
public class ArmTestOmega extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        waitForStart();
        int increment = 0;
        int apressed = 0;

        arm.setTargetPosition(0);

        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
      //  arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
     //   arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //arm.setPower(0.8);

        while (opModeIsActive()){
            telemetry.addData("arm position",arm.getCurrentPosition());
            telemetry.update();




        }
    }
}
