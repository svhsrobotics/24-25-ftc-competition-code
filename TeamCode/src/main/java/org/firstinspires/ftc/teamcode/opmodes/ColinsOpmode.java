package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

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

    @Override

    public void init() {
        frontLeftWheel = hardwareMap.get(DcMotor.class, "FLWheel");
        frontRightWheel = hardwareMap.get(DcMotor.class, "FRWheel");
    }
    @Override
    public void loop() {
        frontLeftWheel.setPower(gamepad1.left_stick_y);
        frontRightWheel.setPower(gamepad1.left_stick_y);
    }

}
