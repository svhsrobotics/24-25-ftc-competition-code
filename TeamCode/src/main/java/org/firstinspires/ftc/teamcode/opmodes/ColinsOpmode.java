package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import java.util.random.RandomGenerator;

import kotlin.random.Random;

@TeleOp

/// random notes pls ignore ///

/// micah prefers shooting end is front ///
/// mr svit says to rename the wheels whatever you want ///
///  when uploading/pushing code remember to not connect to other teams robot lol ///


public class ColinsOpmode extends OpMode {
    private DcMotor frontLeftWheel;
    private DcMotor frontRightWheel;
    private DcMotor backLeftWheel;
    private DcMotor backRightWheel;

    @Override

    public void init() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "leftFront");
        frontRightWheel = hardwareMap.get(DcMotor.class, "rightFront");
        backLeftWheel = hardwareMap.get(DcMotor.class, "leftBack");
        backRightWheel = hardwareMap.get(DcMotor.class, "rightBack");

        frontLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        frontRightWheel.setDirection(DcMotor.Direction.FORWARD);
        backLeftWheel.setDirection(DcMotor.Direction.REVERSE);
        backRightWheel.setDirection(DcMotor.Direction.FORWARD);
    }
    @Override
    public void loop() {
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        frontLeftWheel.setPower(y + x + rx);
        backLeftWheel.setPower(y - x + rx);
        frontRightWheel.setPower(y - x - rx);
        backRightWheel.setPower(y + x - rx);
    }

}
