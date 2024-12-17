package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@TeleOp
@Config
public class PsiTeleOp extends LinearOpMode {


    @Override
    public void runOpMode() throws InterruptedException {






        waitForStart();


        /*int armPos = 300;
        double wristPos = .5;

        ArmState currentState = ArmState.MANUAL;
        int offset = 0;
        double elbowPos = 0.2;
        double pinchLocation = 0.0;
        boolean barHangTriggered = false;

        PIDFController headingController = new PIDFController(HEADING_PID); // TODO: Abstract
        headingController.setInputBounds(0, 2 * Math.PI);
        //double targetHeading = Math.toRadians(180);
        //Pose2d targetHeading = new Pose2d();

        Toggle lockHeading = new Toggle();
        lockHeading.state = true;
        while (opModeIsActive()) {
            drive.update(); // MUST be called every loop cycle so that RoadRunner calculates the pose correctly
            Pose2d poseEstimate = drive.getPoseEstimate();

            double STICK_MULTIPLIER = 1;
            double left_stick_y = (gamepad1.left_stick_y * STICK_MULTIPLIER) + (gamepad2.left_stick_y * STICK_MULTIPLIER);
            double left_stick_x = (gamepad1.left_stick_x * STICK_MULTIPLIER) + (gamepad2.left_stick_x * STICK_MULTIPLIER);
            double right_stick_x = (gamepad1.right_stick_x * STICK_MULTIPLIER) + (gamepad2.right_stick_x * STICK_MULTIPLIER);

            if (config.fieldCentric) {
                // Create a vector from the gamepad x/y inputs
                // Then, rotate that vector by the inverse of that heading
                Vector2d input = new Vector2d(-left_stick_y, -left_stick_x).rotated(-poseEstimate.getHeading()).rotated(-driverPerspective);

                // Pass in the rotated input + right stick value for rotation
                // Rotation is not part of the rotated input thus must be passed in separately
//                if (armPos <= 1500) {
//                    drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), -gamepad1.right_stick_x * 0.5));
//                } else {
//                    drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), gamepad1.right_stick_x * 0.5));
//                }

                double turnPower = -right_stick_x;



//                targetHeading += Math.toRadians(-right_stick_x * 10);

                if (lockHeading.update(gamepad1.right_stick_button || gamepad2.right_stick_button)) {
                    //if (gamepad1.right_stick_button || gamepad2.right_stick_button) {
                    // targetHeading = Math.toRadians(90);
//                    double delta = poseEstimate.getHeading() - Math.toRadians(90);
////                    // Turn towards 0 heading while held
//                    turnPower = -delta * 0.5;

                    headingController.setTargetPosition(Math.toRadians(180));
                    turnPower = headingController.update(poseEstimate.getHeading());

////                    if (poseEstimate.getHeading() > Math.toRadians(90)) {
////                        turnPower = -0.5;
////                    } else if (poseEstimate.getHeading() < Math.toRadians(90)) {
////                        turnPower = 0.5;
////                    } else {
////                        turnPower = 0;
////                    }
                }

                // Normalize target heading to be in range 0 to 2pi
//                if (targetHeading > 2*Math.PI) {
//                    targetHeading = 0;
//                } else if (targetHeading < 0) {
//                    targetHeading = 2*Math.PI;
//                }

                //double delta = poseEstimate.getHeading() - targetHeading;
                //double turnPower = -delta * 1;

                drive.setWeightedDrivePower(new Pose2d(input.getX(), input.getY(), turnPower));


                if (gamepad1.left_stick_button || gamepad2.left_stick_button) {
                    drive.setPoseEstimate(new Pose2d(poseEstimate.getX(), poseEstimate.getY(), Math.toRadians(180)));
                }
            } else if (armPos > ARM_FLIP_OVER && !barHangTriggered) {
                drive.setWeightedDrivePower(new Pose2d((gamepad1.left_stick_y * 0.5) + (gamepad2.left_stick_y * 0.5), (gamepad1.left_stick_x * 0.5) + (gamepad2.left_stick_x * 0.5), (-gamepad1.right_stick_x * 0.5) + (-gamepad2.right_stick_x * 0.5)));
            } else {
                drive.setWeightedDrivePower(new Pose2d((-gamepad1.left_stick_y * 0.5) + (-gamepad2.left_stick_y * 0.5), (-gamepad1.left_stick_x * 0.5) + (-gamepad2.left_stick_x * 0.5), (-gamepad1.right_stick_x * 0.5) + (-gamepad2.right_stick_x * 0.5)));
            }





                rrobot.wristTwistServo.setPosition(0.5);
                //if (gamepad1.right_bumper || gamepad2.right_bumper) {
                if (gamepad1.dpad_right || gamepad2.dpad_right) {
                    wristPos -= 0.008; // UP
                    //} else if (gamepad1.left_bumper || gamepad2.left_bumper) {
                } else if (gamepad1.dpad_left || gamepad2.dpad_left) {
                    wristPos += 0.008; // DOWN
                }
                if (wristPos > 1) wristPos = 1;
                if (wristPos < 0) wristPos = 0;
                rrobot.wristLiftServo.setPosition(wristPos);
                telemetry.addData("WRIST", wristPos);
                telemetry.addData("ARM", armPos);
                telemetry.addData("STATE", currentState);

                // SHOULDER : WILL BE REPLACED AFTER WORM GEAR?
                if (gamepad1.left_trigger + gamepad1.right_trigger + gamepad2.right_trigger + gamepad2.left_trigger > 0.05) {
                    armPos = rrobot.getShoulderCurrentPosition() - (int) (gamepad1.left_trigger * 50) - (int) (gamepad2.left_trigger * 50) + (int) (gamepad1.right_trigger * 50) + (int) (gamepad2.right_trigger * 50);
                    currentState = ArmState.MANUAL;
                }

                // If there is a big difference in positions, turn down the power
                rrobot.setShoulderTargetPosition(armPos);
//                rrobot.shoulderMotor.setTargetPosition(armPos);
//                rrobot.shoulderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//                //if (Math.abs(rrobot.shoulderMotor.getCurrentPosition() - armPos) > SAFETY_ARM_DELTA) {
//                if (armPos < rrobot.shoulderMotor.getCurrentPosition() && armPos < 1000) {
//                    rrobot.shoulderMotor.setPower(SAFETY_ARM_POWER);
//                } else {
//                    rrobot.shoulderMotor.setPower(1);
//                }

                // ELBOW
                //if (gamepad1.dpad_left || gamepad2.dpad_left) {
                if (gamepad1.left_bumper || gamepad2.left_bumper) {
                    elbowPos -= 0.008; // UP
                    //} else if (gamepad1.dpad_right || gamepad2.dpad_right) {
                }   else if (gamepad1.right_bumper || gamepad2.right_bumper) {
                    elbowPos += 0.008; // DOWN
                }
                if (elbowPos > 0.55) elbowPos = 0.55;
                if (elbowPos < 0) elbowPos = 0;
                rrobot.elbowServo.setPosition(elbowPos);
                telemetry.addData("ELBOW", elbowPos);

                if (gamepad1.dpad_down || gamepad2.dpad_down) {
                    pinchLocation = pinchLocation + 0.008;
                } else if (gamepad1.dpad_up || gamepad2.dpad_up) {
                    pinchLocation -= 0.008;
                }
                if (pinchLocation > 0.6) {
                    pinchLocation = 0.6;
                } else if (pinchLocation < 0) {
                    pinchLocation = 0;
                }
                rrobot.pinchServo.setPosition(pinchLocation);
                telemetry.addData("PINCH", pinchLocation);


                // HANG
                if (gamepad2.y) {
                    rrobot.hangMotor.setPower(1.0);
                    barHangTriggered = true;
                    rrobot.planeAngleServo.setPosition(0.3);
                } else if (gamepad2.x) {
                    rrobot.hangMotor.setPower(-1.0);
                } else if (gamepad2.b) {
                    rrobot.planeAngleServo.setPosition(0.3);
                }
                else {
                    rrobot.hangMotor.setPower(0.0);
                }


                if (gamepad1.y) { // RAISED
                    currentState = ArmState.RAISED;
                    armPos = RAISED_ARM;
                    elbowPos = RAISED_ELBOW;
                    wristPos = RAISED_WRIST;
                } else if (gamepad1.x) { // NEUTRAL
                    currentState = ArmState.NEUTRAL;
                    armPos = NEUTRAL_ARM;
                    elbowPos = NEUTRAL_ELBOW;
                    wristPos = NEUTRAL_WRIST;
                } else if (gamepad1.b) { // PREP
                    currentState = ArmState.PREP;
                    armPos = PREP_ARM;
                    elbowPos = PREP_ELBOW;
                    wristPos = PREP_WRIST;
                } else if (gamepad1.a) { // PICKUP
                    currentState = ArmState.PICKUP;
                    armPos = PICKUP_ARM;
                    elbowPos = PICKUP_ELBOW;
                    wristPos = PICKUP_WRIST;
                    pinchLocation = PICKUP_PINCH;
                    rrobot.pinchServo.setPosition(pinchLocation);
                }


                telemetry.addData("OFFSET ", rrobot.shoulderOffset);
                if (gamepad1.back) {
                    rrobot.recalibrateShoulder();
                }

            }

            telemetry.update();
        }
    }

    //public static int ARM_OFFSET = 94;

    public static int RAISED_ARM = 2070;
    public static double RAISED_ELBOW = 0.272;
    public static double RAISED_WRIST = 0.616;

    public static int NEUTRAL_ARM = 113;
    public static double NEUTRAL_ELBOW = 0.248;
    public static double NEUTRAL_WRIST = 0.336;

    public static int PREP_ARM = 2099;
    public static double PREP_ELBOW = 0.032;
    public static double PREP_WRIST = 0.372;

    public static int PICKUP_ARM = -249;
    public static double PICKUP_ELBOW = 0.16;
    public static double PICKUP_WRIST = 0.816;
    public static double PICKUP_PINCH = 0.6;



    public static int ARM_FLIP_OVER = 1350; */
}

}
