package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class GammaDriveMode extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        topLeftMotor = hardwareMap.get(DcMotor.class,"leftFront");
        topRightMotor = hardwareMap.get(DcMotor.class,"rightFront");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"leftBack");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"rightBack");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lift = hardwareMap.get(DcMotor.class,"lift");


        waitForStart();
        double intakeState = 1;
        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CRServo claw = hardwareMap.get(CRServo.class, "intake");

        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx2 = gamepad1.right_stick_x;
            //drive
            topLeftMotor.setPower(y + x + rx2);
            bottomLeftMotor.setPower(y - x + rx2);
            topRightMotor.setPower(y - x - rx2);
            bottomRightMotor.setPower(y + x - rx2);
            //arm
            if (gamepad2.a) {
                arm.setTargetPosition(0);
                arm.setPower(0.5);
            }
            else
            {
                if (gamepad2.b) {
                    arm.setTargetPosition(-850);
                    arm.setPower(0.5);
                }else
                {
                    arm.setPower(0);
                }
            }
            //intake
           if (gamepad2.right_bumper){intakeState=1;}
           if (gamepad2.left_bumper){intakeState=-1;}
           if (gamepad2.y){intakeState=0;}
           claw.setPower(intakeState);
            //lift

//                telemetry.addData("pos", lift.getCurrentPosition());
//                telemetry.update();
                if (gamepad2.dpad_up) {  // Go UP
                    lift.setTargetPosition(-3400);
                    lift.setPower(0.5);
                } else if (gamepad2.dpad_down) {
                    if (lift.getCurrentPosition() < -10) {
                        lift.setPower(0.2);
                        lift.setTargetPosition(lift.getCurrentPosition() + 40);
                        //  sleep(10);
                    }
                } else {
                    // do nothing
                }

    }
    }
}
