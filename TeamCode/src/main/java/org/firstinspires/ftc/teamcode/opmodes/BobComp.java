package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BobComp extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftShoot;
    DcMotor rightShoot;
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

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftShoot = hardwareMap.get(DcMotor.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotor.class, "rightShoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftPush = hardwareMap.get(Servo.class, "leftPush");
        rightPush = hardwareMap.get(Servo.class, "rightPush");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
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

        if (!gamepad1.x) {
            rx = gamepad1.left_stick_y;
            y = -gamepad1.left_stick_x;
            x = -gamepad1.right_stick_x;

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

            leftFront.setPower(0.5 * (y + x + rx));
            leftBack.setPower(0.5 * (y - x + rx));
            rightFront.setPower(0.5 * (y - x - rx));
            rightBack.setPower(0.5 * (y + x - rx));

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
            telemetry.addData("Intake Power: ", (gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));

            if (!gamepad1.left_bumper) {
                leftPush.setPosition(0.7);
                rightPush.setPosition(0.7);
            }
            else {
                leftPush.setPosition(0.14);
                rightPush.setPosition(0.14);
            }
        }
        else {
            leftPush.setPosition(0.5);
            rightPush.setPosition(0.5);
            leftShoot.setPower(-1);
            rightShoot.setPower(-1);
            intake.setPower(-1);
            //test this
        }
    }
}