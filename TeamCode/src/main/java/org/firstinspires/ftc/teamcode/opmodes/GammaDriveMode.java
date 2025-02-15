package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class GammaDriveMode extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    private Servo wrist;

    private double drivespeedmultiplier = 0.5;
    @Override
    public void runOpMode() throws InterruptedException {
        topLeftMotor = hardwareMap.get(DcMotor.class,"leftFront");
        topRightMotor = hardwareMap.get(DcMotor.class,"rightFront");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"leftBack");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"rightBack");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lift = hardwareMap.get(DcMotor.class,"lift");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        waitForStart();
        double intakeState = 0;
        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CRServo claw = hardwareMap.get(CRServo.class, "intake");

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.scaleRange(0.1,0.8);

        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx2 = gamepad1.right_stick_x;
            //drive
            topLeftMotor.setPower((y + x + rx2)*drivespeedmultiplier);
            bottomLeftMotor.setPower((y - x + rx2)*drivespeedmultiplier);
            topRightMotor.setPower((y - x - rx2)*drivespeedmultiplier);
            bottomRightMotor.setPower((y + x - rx2)*drivespeedmultiplier);
            //arm
            if (gamepad2.b) {
                arm.setTargetPosition(0);
                arm.setPower(2);
            }
            else
            {
                if (gamepad2.a) {
                    arm.setTargetPosition(-2000);
                    arm.setPower(0.6);
                }else
                {
                    arm.setPower(0);
                }
            }
            //intake
           if (gamepad1.right_bumper){intakeState=1;}
           if (gamepad1.left_bumper){intakeState=-1;}
           if (!gamepad1.right_bumper &&!gamepad1.left_bumper){intakeState=0;}
           claw.setPower(intakeState);
            //lift
//-2193
//                telemetry.addData("pos", lift.getCurrentPosition());
//                telemetry.update();
                if (gamepad2.dpad_up) {  // Go UP
                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    lift.setTargetPosition(-4200);
                    lift.setPower(1);
                } else if (gamepad2.dpad_down) {
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    lift.setPower(0);
                } else if (gamepad2.dpad_right) {
//                    lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                    lift.setTargetPosition(-2300);
//                    lift.setPower(0.5);
                } else {
                    // do nothing
                }

                //wrist
            if (gamepad1.x) {
                if (wrist.getPosition()>0.1) {
                    wrist.setPosition(wrist.getPosition() - 0.002);
                }
            } else if (gamepad1.y) {
                if (wrist.getPosition()<0.8) {
                    wrist.setPosition(wrist.getPosition() + 0.002);
                }
            }
            else {

            }


    }
    }
}
