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
    double y;
    double x;
    double rx;
    boolean dPadPressed;
    boolean shouldShoot;
    double heading;
    double targetHeading;
    double distance;
    VoltageSensor batteryVoltageSensor;
    AprilTagProcessor tagProcessor;

    VisionPortal visionPortal;

    List<AprilTagDetection> witnessedTags = new ArrayList<>();
    Telemetry tagInfo;
    ElapsedTime never = new ElapsedTime();

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
        rightPush.setDirection(Servo.Direction.REVERSE);
        distance = 0;
        leftPush.setPosition(0.86);
        rightPush.setPosition(0.86);

        waitForStart();

        leftFront.setPower(-0.25);
        leftBack.setPower(-0.25);
        rightFront.setPower(-0.25);
        rightBack.setPower(-0.25);

        while (distance < 160) {
            for (AprilTagDetection detection : witnessedTags) {
                if (detection.metadata.id == 20) {
                    targetHeading = detection.ftcPose.yaw;
                    telemetry.addData("Target Distance: ", detection.ftcPose.range);
                    distance = detection.ftcPose.range;
                }
            }
        }
        while(targetHeading < -8
                || targetHeading > -5) {
            if (targetHeading > -5) {
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
                rightFront.setPower(-0.1);
                rightBack.setPower(-0.1);
            } else if (targetHeading < -8) {
                leftFront.setPower(-0.1);
                leftBack.setPower(-0.1);
                rightFront.setPower(0.1);
                rightBack.setPower(0.1);
            } else {
                leftFront.setPower(0);
                leftBack.setPower(0);
                rightFront.setPower(0);
                rightBack.setPower(0);
            }
            for (AprilTagDetection detection : witnessedTags) {
                if (detection.metadata.id == 20) {
                    targetHeading = detection.ftcPose.yaw;
                    telemetry.addData("Target Distance: ", detection.ftcPose.range);
                    distance = detection.ftcPose.range;
                }
            }
        }

        if (distance < 170) {
            shoot = 750;
        } else if (distance < 200) {
            shoot = 760;
        } else if (distance < 210) {
            shoot = 775;
        } else if (distance < 217) {
            shoot = 790;
        }

        leftShoot.setVelocity(shoot);
        rightShoot.setVelocity(shoot);

        sleep (5000);

        leftPush.setPosition(0.14);
        rightPush.setPosition(0.84);

        sleep (1000);

        leftFront.setPower(-0.3);
        leftBack.setPower(0.3);
        rightFront.setPower(0.3);
        rightBack.setPower(-0.3);

        sleep(5000);

        leftFront.setPower(0);
        leftBack.setPower(0);
        rightFront.setPower(0);
        rightBack.setPower(0);
    }
}
