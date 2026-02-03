package org.firstinspires.ftc.teamcode.opmodes;

import android.graphics.Color;
import android.util.Size;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

import java.util.ArrayList;
import java.util.List;
import java.util.Objects;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class BobComp extends OpMode {

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
    boolean targetSwapping;
    boolean shouldShoot;
    boolean targetSeen;
    double targetHeading;
    double targetID;
    double distance;
    VoltageSensor batteryVoltageSensor;
    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    List<AprilTagDetection> witnessedTags = new ArrayList<>();
    Telemetry tagInfo;

    @Override
    public void init() {
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

    }

    @Override
    public void start() {
        leftShoot.setTargetPosition(0);
        rightShoot.setTargetPosition(0);
        dPadPressed = false;
        targetSwapping = false;
        shouldShoot = false;
        targetHeading = 0;
        targetID = 20;
    }

    @Override
    public void loop() {
        telemetry.addData("Shooting Power", shoot);
        if (leftShoot.getVelocity() < shoot + 50
                && leftShoot.getVelocity() > shoot - 50
                && rightShoot.getVelocity() < shoot + 50
                && rightShoot.getVelocity() > shoot - 50) {
            telemetry.addLine("Launcher Powered!");
        }
        else {
            telemetry.addLine("Launcher is not powered!");
        }
        //telemetry.addData("left velocity", leftShoot.getVelocity());
        //telemetry.addData("right velocity", rightShoot.getVelocity());

        y = -gamepad1.left_stick_y;
        rx = gamepad1.left_stick_x;
        x = gamepad1.right_stick_x;

        if (gamepad1.dpad_up && !dPadPressed) {
            dPadPressed = true;
            shoot += 10;
        }
        if (gamepad1.dpad_down && !dPadPressed) {
            dPadPressed = true;
            shoot -= 10;
        }
        if (gamepad1.dpad_right && !dPadPressed) {
            dPadPressed = true;
            shoot += 50;
        }
        if (gamepad1.dpad_left && !dPadPressed) {
            dPadPressed = true;
            shoot -= 50;
        }
        if (!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
            dPadPressed = false;
        }


        if (gamepad1.left_bumper) {
            shoot = 750;
        }
        if (gamepad1.right_bumper) {
            shoot = 950;
        }

        if (!gamepad1.x) {
            leftPush.setPosition(0.86);
            rightPush.setPosition(0.3);
        } else {
            leftPush.setPosition(0.14);
            rightPush.setPosition(0.84);
        }

        if (gamepad1.a) {
            shouldShoot = true;
        }
        if (gamepad1.b) {
            shouldShoot = false;
        }

        if (shouldShoot) {
            leftShoot.setVelocity(shoot);
            rightShoot.setVelocity(shoot);
        }
        else {
            leftShoot.setVelocity(0);
            rightShoot.setVelocity(0);
        }

        intake.setPower((gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));
        //telemetry.addData("Intake Power: ", (gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));

        if (!targetSwapping) {
            if (gamepad1.left_stick_button
                    && gamepad1.right_stick_button) {
                targetSwapping = true;
                if (targetID == 20) {
                    targetID = 24;
                }
                else {
                    targetID = 20;
                }
            }
        }
        else {
            if (!gamepad1.left_stick_button
                    && !gamepad1.right_stick_button) {
                targetSwapping = false;
            }
        }
        if(targetID == 20) {
            telemetry.addLine("Target: BLUE");
        }
        else {
            telemetry.addLine("Target: RED");
            gamepad1.setLedColor(255, 0, 0, 300);
        }

        targetSeen = false;
        witnessedTags = tagProcessor.getDetections();
        for (AprilTagDetection detection : witnessedTags) {
            if (detection.metadata.id == targetID) {
                targetHeading = detection.ftcPose.yaw;
                telemetry.addData("Target Distance: ", detection.ftcPose.range);
                distance = detection.ftcPose.range;
                targetSeen = true;
            }
        }
        if (!targetSeen) {
            distance = 999;
            targetHeading = 0;
            telemetry.addLine("Target Distance: N/A");
        }

        if (gamepad1.y && targetID == 20) {
            if (targetHeading > -6) {
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
                rightFront.setPower(-0.1);
                rightBack.setPower(-0.1);
            } else if (targetHeading < -7) {
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
        }
        else if (gamepad1.y && targetID == 24) {
            if (targetHeading > 6) {
                leftFront.setPower(0.1);
                leftBack.setPower(0.1);
                rightFront.setPower(-0.1);
                rightBack.setPower(-0.1);
            } else if (targetHeading < 5) {
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
        }
        else {
            leftFront.setPower(0.85 * (y + x + rx));
            leftBack.setPower(0.85 * (y - x + rx));
            rightFront.setPower(0.85 * (y - x - rx));
            rightBack.setPower(0.85 * (y + x - rx));
        }

        //telemetry.addData("Left shooter current: ", leftShoot.getCurrent(CurrentUnit.MILLIAMPS));
        //telemetry.addData("Right shooter current: ", rightShoot.getCurrent(CurrentUnit.MILLIAMPS));
        double voltage = batteryVoltageSensor.getVoltage();
        //telemetry.addData("Battery Voltage (V)", "%.2f", voltage);
        //telemetry.addData("imu", heading);
        //telemetry.addData("targetHeading", targetHeading);
        if ((targetHeading < -5
                && targetHeading > -8
                && targetID == 20)
                || (targetHeading < 7
                && targetHeading > 4
                && targetID == 24)) {
            telemetry.addLine("Target Locked");
            gamepad1.rumble(12);
            if (gamepad1.y) {
                if (distance < 170) {
                    shoot = 750;
                } else if (distance < 200) {
                    shoot = 760;
                } else if (distance < 210) {
                    shoot = 775;
                } else if (distance < 217) {
                    shoot = 790;
                }
            }
        }
        else{
            telemetry.addLine("Target Out of Sight");
        }
        telemetry.update();
    }
}
