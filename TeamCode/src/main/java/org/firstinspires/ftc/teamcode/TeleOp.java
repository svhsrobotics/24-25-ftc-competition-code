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


        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double elbowPosition = 0;
        boolean holdingPosition = false;

        waitForStart();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            x = gamepad1.right_stick_x;
            rx = gamepad1.left_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

            if (gamepad1.right_bumper) {
                leftLiftMotor.setPower(1);
                //rightLiftMotor.setPower(0.2);
                holdingPosition = false;
            } else if (gamepad1.left_bumper) {
                leftLiftMotor.setPower(0.01);
                //rightLiftMotor.setPower(0.01);
                holdingPosition = false;
            } else if (!holdingPosition) {
                leftLiftMotor.setPower(0);
                //rightLiftMotor.setPower(0);
            } else {
                leftLiftMotor.setPower(0.05);
                //rightLiftMotor.setPower(0.05);

            }

            if (gamepad1.right_trigger > 0.1) {
                intakeSlide.setPower(0.5);
            } else if (gamepad1.left_trigger > 0.1) {
                intakeSlide.setPower(-0.5);
            } else {
                intakeSlide.setPower(0);
            }

            if (gamepad1.b) {
                intakeElbow.setPosition(0);
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


            if (gamepad2.a) {
                outtakeClaw.setPosition(1);
                intakeElbow.setPosition(.27);
                while (intakeSlide.getCurrentPosition() > .01) {
                        intakeSlide.setPower(-.5);
                }
                outtakeClaw.setPosition(0);
                while (intakeClaw.getPosition() < .90) {
                    intakeClaw.setPosition(intakeClaw.getPosition() + .01);
                }
                outtakeElbow.setPosition(.6);
                intakeElbow.setPosition(.8);

            }

            if (gamepad2.b) {
                leftLiftMotor.setTargetPosition(300);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(1);
            }
            if (leftLiftMotor.getCurrentPosition() > 1000) {
                leftLiftMotor.setPower(0.0);
                //rightLiftMotor.setPower(0.0);
            }

            liftPosition = leftLiftMotor.getCurrentPosition();


            telemetry.addData("LIFT POSITION", liftPosition);
            telemetry.addData("VIPER SLIDE POSITION", intakeSlide.getCurrentPosition());
            telemetry.addData("INTAKE ELBOW POSITION", elbowPosition);
            telemetry.update();

        }
    }

}
