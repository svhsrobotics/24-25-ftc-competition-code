package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class GammaOpMode extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        DcMotor frontLeftMotor = hardwareMap.dcMotor.get("frontLeftMotor");
        DcMotor backLeftMotor = hardwareMap.dcMotor.get("backLeftMotor");
        DcMotor frontRightMotor = hardwareMap.dcMotor.get("frontRightMotor");
        DcMotor backRightMotor = hardwareMap.dcMotor.get("backRightMotor");
        DcMotor LeftShooter = hardwareMap.dcMotor.get("LeftShooter");
        DcMotor RightShooter = hardwareMap.dcMotor.get("RightShooter");
        DcMotor Intake = hardwareMap.dcMotor.get("Intake")    ;
        frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftShooter.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {
            //kintematic motor equations
            double y = -gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * 1.1;
            double rx = gamepad1.right_stick_x;
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;
            frontLeftMotor.setPower(frontLeftPower);
            backLeftMotor.setPower(backLeftPower);
            frontRightMotor.setPower(frontRightPower);
            backRightMotor.setPower(backRightPower);


            if (gamepad2.a) {//start shooter
                LeftShooter.setPower(1);
                RightShooter.setPower(1);
            }
            if (gamepad2.b) {//start intake
                Intake.setPower(1);
            }
            if (gamepad2.dpad_up) {//start both
                LeftShooter.setPower(1);
                RightShooter.setPower(1);
                Intake.setPower(1);
            }
            if (gamepad2.dpad_down) { // sets everything backwards - only for unjamming if jammed
                LeftShooter.setPower(-1);
                RightShooter.setPower(-1);
                Intake.setPower(-1);
            }
        }
    }
}