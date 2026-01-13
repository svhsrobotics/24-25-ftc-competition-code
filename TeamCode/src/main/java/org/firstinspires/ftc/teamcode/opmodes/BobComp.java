package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
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

import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    boolean shouldShoot;
    double heading;
    VoltageSensor batteryVoltageSensor;

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

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftShoot.setDirection(DcMotor.Direction.FORWARD);
        rightShoot.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
        leftPush.setDirection(Servo.Direction.FORWARD);
        rightPush.setDirection(Servo.Direction.REVERSE);
    }

    @Override
    public void start () {
        leftShoot.setTargetPosition(0);
        rightShoot.setTargetPosition(0);
    }

    @Override
    public void loop () {
        telemetry.addData("Shooting Power", shoot);

            y = -gamepad1.left_stick_y;
            rx = gamepad1.left_stick_x;
            x = gamepad1.right_stick_x;

            if (gamepad1.dpad_up && !dPadPressed) {
                dPadPressed = true;
                shoot += 0.01;
            }
            if (gamepad1.dpad_down && !dPadPressed) {
                dPadPressed = true;
                shoot -= 0.01;
            }
            if (gamepad1.dpad_right && !dPadPressed) {
                dPadPressed = true;
                shoot += 0.05;
            }
            if (gamepad1.dpad_left && !dPadPressed) {
                dPadPressed = true;
                shoot -= 0.05;
            }
            if (!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
                dPadPressed = false;
            }

            leftFront.setPower(0.85 * (y + x + rx));
            leftBack.setPower(0.85 * (y - x + rx));
            rightFront.setPower(0.85 * (y - x - rx));
            rightBack.setPower(0.85 * (y + x - rx));

            if (gamepad1.a) {
                shouldShoot = true;
            }
            if (gamepad1.b) {
                shouldShoot = false;
            }

            if (shouldShoot) {
                leftShoot.setPower(shoot);
                rightShoot.setPower(shoot);
            }
            if (shouldShoot) {
                leftShoot.setPower(0);
                rightShoot.setPower(0);
            }

            intake.setPower((gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));
            if(gamepad1.right_stick_button) {
                shoot = 0.53;
            }
            telemetry.addData("Intake Power: ", (gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));

            if (!gamepad1.left_bumper) {
                leftPush.setPosition(0.7);
                rightPush.setPosition(0.7);
            }
            else {
                leftPush.setPosition(0.14);
                rightPush.setPosition(0.14);
            }

            if (gamepad1.y) {
                heading = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                //double targetHeading
                //complete auto targeting
            }
        telemetry.addData("Left shooter current: ", leftShoot.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("Right shooter current: ", rightShoot.getCurrent(CurrentUnit.MILLIAMPS));
        double voltage = batteryVoltageSensor.getVoltage();
        telemetry.addData("Battery Voltage (V)", "%.2f", voltage);
        telemetry.update();
    }
}