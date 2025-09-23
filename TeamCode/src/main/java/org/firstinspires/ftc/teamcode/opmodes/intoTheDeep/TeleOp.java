package org.firstinspires.ftc.teamcode.opmodes.intoTheDeep;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
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

    private Servo sweep1;

    private Servo sweep2;

    private boolean drivePressed = false;

    private boolean mode = false;

    private boolean rightpressed = false;


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

        sweep1 = hardwareMap.get(Servo.class, "sweep1");
        sweep2 = hardwareMap.get(Servo.class, "sweep2");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);




        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
//        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double elbowPosition = 0;
        boolean holdingPosition = false;
        driveSlow = false;


        waitForStart();
        while (opModeIsActive()) {
            sweep1.setPosition(.38);
            sweep2.setPosition(.51);
            y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            if (gamepad1.dpad_down&&!driveSlow && !drivePressed) {
                drivePressed = true;
                driveSlow = true;
            } else if (gamepad1.dpad_down&&driveSlow && !drivePressed) {
                drivePressed = true;
                driveSlow = false;
            }

            if (!gamepad1.dpad_down) {
                drivePressed = false;
            }


            if (gamepad1.dpad_right&&!rightpressed&&!mode) {
                mode = true;
                rightpressed = true;
            } else if (gamepad1.dpad_right&&mode&&!rightpressed) {
                mode = false;
                rightpressed = true;
            }

            if (!gamepad1.dpad_right) {
                rightpressed = false;
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



            if (gamepad2.x) {
                leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            }

            if (gamepad2.b) {
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setPower(1);
                //rightLiftMotor.setPower(0.2);
            } else if (gamepad2.a) {
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setPower(-.3);
                //rightLiftMotor.setPower(0.01);
            } else if (leftLiftMotor.getMode() == DcMotor.RunMode.RUN_USING_ENCODER) {
                leftLiftMotor.setPower(0);
            }

            if (gamepad1.right_trigger > 0.1 && intakeSlide.getCurrentPosition() < 800&&!mode) {
                intakeSlide.setPower(0.5);
            } else if (gamepad1.right_trigger > 0.1&&mode) {
                outtakeElbow.setPosition(outtakeElbow.getPosition()+0.01);
            }
            if (gamepad1.left_trigger > 0.1) {
                intakeSlide.setPower(-0.5);
            } else if (gamepad1.left_trigger > 0.1&&mode) {
                outtakeElbow.setPosition(outtakeElbow.getPosition()-0.01);
            }

            if (!(gamepad1.right_trigger > 0.1) && !(gamepad1.left_trigger > 0.1)){
                intakeSlide.setPower(0);
            }

            if (gamepad1.b) {
                intakeElbow.setPosition(.8);
            }
            if (gamepad1.y) {
                intakeElbow.setPosition(.13);
            }
            if (gamepad1.x) {
                intakeElbow.setPosition(.95);
            }
            if (gamepad1.dpad_left) {
                intakeElbow.setPosition(.395);
            }
            if (gamepad1.dpad_up) {
                intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }
            if (gamepad1.right_bumper && !mode) {
                intakeClaw.setPosition(0);
            } else if (gamepad1.right_bumper) {
                intakeElbow.setPosition(intakeElbow.getPosition() - 0.01);
            }
            if (gamepad1.left_bumper&&!mode) {
                intakeClaw.setPosition(1);
            } else if (gamepad1.left_bumper) {
                intakeElbow.setPosition(intakeElbow.getPosition() + 0.01);
            }


            if (gamepad2.right_bumper) {
                outtakeClaw.setPosition(.8);
            }

            if (gamepad2.left_bumper) {
                outtakeClaw.setPosition(.2);
            }

            if (gamepad2.dpad_up) {

                outtakeElbow.setPosition(.49);
            }

            if (gamepad2.dpad_right) {
                outtakeElbow.setPosition(.465);
            }
            if (gamepad2.dpad_down) {
                outtakeElbow.setPosition(.7);
            }
            if (gamepad2.dpad_left) {
                outtakeElbow.setPosition(.6);
            }


            if (gamepad1.a && !(leftLiftMotor.getCurrentPosition() > 300) && !(x>.1 || y>.1 || rx>.1 || x<-.1 || y<-.1 || rx<-.1)) {
                while (intakeSlide.getCurrentPosition() < 800) {
                    intakeSlide.setPower(.3);
                }

                outtakeClaw.setPosition(.465);
                intakeElbow.setPosition(.13);
                while (intakeSlide.getCurrentPosition() > 250) {
                        intakeSlide.setPower(-.20);
                }

                while (intakeElbow.getPosition() > .5 || intakeElbow.getPosition() > .5) {
                        telemetry.addData("intakeElbowPos", intakeElbow.getPosition());
                        telemetry.update();
                }
                outtakeClaw.setPosition(.15);
                while (intakeClaw.getPosition() < .90) {
                    intakeClaw.setPosition(intakeClaw.getPosition() + .005);
                }
                intakeElbow.setPosition(.7);

            }


            //TODO: FIX ELBOW INTAKE POSITION
            if (gamepad2.y && !(intakeElbow.getPosition() < .5)) {
                leftLiftMotor.setTargetPosition(2600);
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                leftLiftMotor.setPower(1);
                outtakeElbow.setPosition(.7);
                int z = 0;
                while (leftLiftMotor.getCurrentPosition() < 2600) {
                    z++;
                }
                outtakeClaw.setPosition(1);
                while (outtakeClaw.getPosition() < 1) {
                    z++;
                }
                outtakeElbow.setPosition(.465);
                while (outtakeElbow.getPosition()>.465) {
                    z++;
                }
                leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                leftLiftMotor.setPower(0);

            }
//            if (leftLiftMotor.getCurrentPosition() > 1000) {
//                leftLiftMotor.setPower(0.0);
//                //rightLiftMotor.setPower(0.0);
//            }

            liftPosition = leftLiftMotor.getCurrentPosition();


            telemetry.addData("LIFT POSITION", liftPosition);
            telemetry.addData("VIPER SLIDE POSITION", intakeSlide.getCurrentPosition());
            telemetry.addData("INTAKE ELBOW POSITION", elbowPosition);
            telemetry.addData("DRIVE SLOW: ", driveSlow);
            telemetry.addData("X: ", x);
            telemetry.addData("Y: ", y);
            telemetry.addData("RX: ", rx);
            telemetry.addData("INTAKE_ELBOW: ", intakeElbow.getPosition());
            telemetry.update();

        }
    }

}
