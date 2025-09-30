package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;


@Config
@TeleOp

public class BasicDriveMode extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
        DcMotor topLeftMotor = hardwareMap.get(DcMotor.class,"FL");
        DcMotor topRightMotor = hardwareMap.get(DcMotor.class,"FR");
        DcMotor bottomLeftMotor = hardwareMap.get(DcMotor.class,"BL");
        DcMotor bottomRightMotor = hardwareMap.get(DcMotor.class,"BR");
        while (opModeIsActive()) {
            double y = -gamepad1.left_stick_y; // Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx2 = gamepad1.right_stick_x;

            topLeftMotor.setPower(y+ x + rx2);
            bottomLeftMotor.setPower(y - x + rx2);
            topRightMotor.setPower(y - x - rx2);
            bottomRightMotor.setPower(y + x - rx2);
        }
    }
}
