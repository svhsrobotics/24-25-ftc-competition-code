package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftLiftMotor;
    //private DcMotor rightLiftMotor;
    private DcMotor intakeSlide;
    private Servo intakeElbow;
    private Servo intakeClaw;
    private Servo outtakeElbow;
    private Servo outtakeClaw;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private double liftPosition;
    private boolean driveSlow;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift");
        //rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift");
        intakeSlide = hardwareMap.get(DcMotor.class, "intake_slide");
        intakeElbow = hardwareMap.get(Servo.class, "intake_wrist");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        outtakeElbow = hardwareMap.get(Servo.class, "deposit_wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtake_grab");


        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double elbowPosition = 0;
        boolean holdingPosition = false;
        driveSlow = false;


        waitForStart();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            x = gamepad1.right_stick_x;
            rx = gamepad1.left_stick_x;

            if (gamepad2.y&&!driveSlow) {
                driveSlow = true;
            } else if (gamepad2.y && driveSlow) {
                driveSlow = false;
            }


            if (driveSlow) {
                rightBackMotor.setPower((y + x + rx)*.5);
                rightFrontMotor.setPower((y - x + rx)*.5);
                leftBackMotor.setPower((y - x - rx)*.5);
                leftFrontMotor.setPower((y + x - rx)*.5);
            } else {
                rightBackMotor.setPower((y + x + rx));
                rightFrontMotor.setPower((y - x + rx));
                leftBackMotor.setPower((y - x - rx));
                leftFrontMotor.setPower((y + x - rx));
            }

            if (gamepad1.right_trigger >0.1) {
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setPower(gamepad1.right_trigger);
                //rightLiftMotor.setPower(0.2);
            } else if (gamepad1.left_trigger > 0.1) {
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setPower(gamepad1.left_trigger * -1);
                //rightLiftMotor.setPower(0.01);
            } else if (leftLiftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                leftLiftMotor.setPower(0);
            }

            if (gamepad1.right_bumper && intakeSlide.getCurrentPosition() < 200) {
                intakeSlide.setPower(0.5);
            } else if (gamepad1.left_bumper) {
                intakeSlide.setPower(-0.5);
            } else {
                intakeSlide.setPower(0);
            }

            if (gamepad1.b) {
                intakeElbow.setPosition(.7);
            }
            if (gamepad1.y) {
                intakeElbow.setPosition(.27);
            }
            if (gamepad1.x) {
                intakeElbow.setPosition(.8);
            }
            if (gamepad1.dpad_down) {
                intakeClaw.setPosition(0);
            }
            if (gamepad1.dpad_up) {
                intakeClaw.setPosition(1);
            }

            if (gamepad1.dpad_left) {
                outtakeClaw.setPosition(1);
            }

            if (gamepad1.dpad_right) {
                outtakeClaw.setPosition(0);
            }

            if (gamepad2.left_bumper) {

                outtakeElbow.setPosition(.5);
            }

            if (gamepad2.right_bumper) {

                outtakeElbow.setPosition(0);
            }
            if (gamepad2.right_bumper) {
                outtakeElbow.setPosition(1);
            }


            if (gamepad2.a) {
                while (intakeSlide.getCurrentPosition() < 198) {
                    intakeSlide.setPower(.2);
                }
                outtakeClaw.setPosition(1);
                intakeElbow.setPosition(.27);
                while (intakeSlide.getCurrentPosition() > .01) {
                        intakeSlide.setPower(-.2);
                }
                while (intakeElbow.getPosition() > .29 || intakeElbow.getPosition() < .26) {
                        telemetry.addData("intakeElbowPos", intakeElbow.getPosition());
                        telemetry.update();
                }
                outtakeClaw.setPosition(0);
                while (intakeClaw.getPosition() < .90) {
                    intakeClaw.setPosition(intakeClaw.getPosition() + .01);
                }
                intakeElbow.setPosition(.5);

            }

            if (gamepad2.b) {
                leftLiftMotor.setTargetPosition(2000);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(1);
            }
//            if (leftLiftMotor.getCurrentPosition() > 1000) {
//                leftLiftMotor.setPower(0.0);
//                //rightLiftMotor.setPower(0.0);
//            }

            liftPosition = leftLiftMotor.getCurrentPosition();


            telemetry.addData("LIFT POSITION", liftPosition);
            telemetry.addData("VIPER SLIDE POSITION", intakeSlide.getCurrentPosition());
            telemetry.addData("INTAKE ELBOW POSITION", elbowPosition);
            telemetry.update();

        }
    }

}
