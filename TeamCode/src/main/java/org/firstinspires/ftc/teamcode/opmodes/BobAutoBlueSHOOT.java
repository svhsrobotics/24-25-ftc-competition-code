package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class BobAutoBlueSHOOT extends LinearOpMode {
    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotorEx leftShoot;
    DcMotorEx rightShoot;
    DcMotor intake;
    Servo leftPush;
    Servo rightPush;
    IMU imu;
    double shoot;
    boolean targetSeen;
    double targetHeading;
    double distance;
    VoltageSensor batteryVoltageSensor;
    AprilTagProcessor tagProcessor;

    VisionPortal visionPortal;

    List<AprilTagDetection> witnessedTags = new ArrayList<>();
    Telemetry tagInfo;
    ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftShoot = hardwareMap.get(DcMotorEx.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotorEx.class, "rightShoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPush = hardwareMap.get(Servo.class, "leftPush");
        rightPush = hardwareMap.get(Servo.class, "rightPush");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        batteryVoltageSensor = hardwareMap.voltageSensor.iterator().next();

        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftShoot.setDirection(DcMotor.Direction.FORWARD);
        rightShoot.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftPush.setDirection(Servo.Direction.FORWARD);
        rightPush.setDirection(Servo.Direction.FORWARD);
        distance = 0;
        targetSeen = false;

        waitForStart();
        leftPush.setPosition(0.86);
        rightPush.setPosition(0.3);
        timer.reset();

        leftFront.setPower(-0.3);
        leftBack.setPower(-0.3);
        rightFront.setPower(-0.3);
        rightBack.setPower(-0.3);

        while (opModeIsActive() && distance < 160) {
            witnessedTags = tagProcessor.getDetections();
            for (AprilTagDetection detection : witnessedTags) {
                if (detection.metadata != null
                        && detection.metadata.id == 20) {
                    targetHeading = detection.ftcPose.bearing;
                    telemetry.addData("Target Distance: ", detection.ftcPose.range);
                    telemetry.addData("Target Heading: ", targetHeading);
                    distance = detection.ftcPose.range;
                }
            }
            telemetry.addData("Time elapsed: ", timer);
            telemetry.update();
        }
        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        proportionalTargeting();

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);

        if (distance < 170) {
            shoot = 750;
        } else if (distance < 200) {
            shoot = 760;
        }
        else {
            while (distance > 160) {
                leftFront.setPower(-0.2);
                leftBack.setPower(-0.2);
                rightFront.setPower(-0.2);
                rightBack.setPower(-0.2);
            }
            if (distance < 170) {
                shoot = 750;
            } else if (distance < 200) {
                shoot = 760;
            }
        }

        leftShoot.setVelocity(shoot);
        rightShoot.setVelocity(shoot);

        timer.reset();
        sleep(8000);
        /*while (leftShoot.getVelocity() > shoot + 50
                || leftShoot.getVelocity() < shoot - 50
                || rightShoot.getVelocity() > shoot + 50
                || rightShoot.getVelocity() < shoot - 50
                || timer.seconds() < 1) {
            if (leftShoot.getVelocity() < shoot + 50
                    && leftShoot.getVelocity() > shoot - 50
                    && rightShoot.getVelocity() < shoot + 50
                    && rightShoot.getVelocity() > shoot - 50
                    && timer.seconds() > 1) {
                timer.reset();
                sleep(1000);
            }
            telemetry.addLine("Not Powered");
            telemetry.update();
        }
         */

        intake.setPower(-0.7);
        leftPush.setPosition(0.14);
        rightPush.setPosition(0.84);

        sleep(3000);

        intake.setPower(0);
        leftShoot.setVelocity(0);
        rightShoot.setVelocity(0);

        leftPush.setPosition(0.86);
        rightPush.setPosition(0.3);

        leftFront.setPower(-0.3 + 0.1);
        leftBack.setPower(0.3 + 0.1);
        rightFront.setPower(0.3 + 0.1);
        rightBack.setPower(-0.3 + 0.1);

        sleep(3000);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }

    private void proportionalTargeting() {
        witnessedTags = tagProcessor.getDetections();
        for (AprilTagDetection detection : witnessedTags) {
            if (detection.metadata != null
                    && detection.metadata.id == 20) {
                targetHeading = detection.ftcPose.bearing;
                telemetry.addData("Target Distance: ", detection.ftcPose.range);
                telemetry.addData("Target Heading: ", targetHeading);
                distance = detection.ftcPose.range;
                targetSeen = true;
            }
            telemetry.update();
        }
        while (targetHeading < -0.75
                || targetHeading > 1.75
                && opModeIsActive()) {
            witnessedTags = tagProcessor.getDetections();
            targetSeen = false;
            for (AprilTagDetection detection : witnessedTags) {
                if (detection.metadata != null
                        && detection.metadata.id == 20) {
                    targetHeading = detection.ftcPose.bearing;
                    telemetry.addData("Target Distance: ", detection.ftcPose.range);
                    telemetry.addData("Target Heading: ", targetHeading);
                    distance = detection.ftcPose.range;
                    targetSeen = true;
                }
                telemetry.update();
            }
            if (targetSeen) {
                double turnPower = (targetHeading - 0.5) * 0.05;
                leftFront.setPower(turnPower);
                leftBack.setPower(turnPower);
                rightFront.setPower(-turnPower);
                rightBack.setPower(-turnPower);
            }
            else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
        }
    }
}