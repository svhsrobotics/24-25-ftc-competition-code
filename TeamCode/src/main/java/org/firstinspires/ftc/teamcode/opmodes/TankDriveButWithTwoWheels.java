package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

@Config
@TeleOp
public class TankDriveButWithTwoWheels extends LinearOpMode {


    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotor intake;
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;

    private Servo gateServo;
    @Override
    public void runOpMode() {

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch2=hardwareMap.get(DcMotor.class, "launch2");
        launch=hardwareMap.get(DcMotor.class, "launch1");
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo.setDirection(Servo.Direction.REVERSE);
        waitForStart();

        while (opModeIsActive()) {

            launch2.setPower(gamepad1.left_trigger);
            launch.setPower(gamepad1.left_trigger);
            intake.setPower(gamepad1.right_trigger);
           right.setPower((gamepad1.right_stick_x+ gamepad1.left_stick_y));
           left.setPower((gamepad1.right_stick_x -gamepad1.left_stick_y));
           if (gamepad1.x) {
               gateServo.setPosition(0);

           }
           if (gamepad1.b) {
               gateServo.setPosition(0.9);
           }
        }



        /*
        waitForStart();

        while (opModeIsActive()) {
            topLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
            topRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
            bottomLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
            bottomRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
            topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

            topRightMotor.setPower(gamepad1.right_stick_y + gamepad1.left_stick_x);
            topLeftMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);

            bottomRightMotor.setPower(gamepad1.right_stick_y + gamepad1.left_stick_x);
            bottomLeftMotor.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
        }
        */
}}