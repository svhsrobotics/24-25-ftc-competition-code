package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@TeleOp
public class lifttestomega extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor lift = hardwareMap.get(DcMotor.class,"lift");
        waitForStart();
        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
      while (opModeIsActive()){
                telemetry.addData("pos", lift.getCurrentPosition());
                telemetry.update();
//                if (gamepad1.a) {  // Go UP
//                    lift.setTargetPosition(-3200);
//                    lift.setPower(0.5);
//                } else if (gamepad1.b) {
//                    if (lift.getCurrentPosition() < -10) {
//                        lift.setPower(0.2);
//                        lift.setTargetPosition(lift.getCurrentPosition() + 35);
//                      //  sleep(10);
//                    }
//                } else {
//                    // do nothing
//             }
       }
    }
}
