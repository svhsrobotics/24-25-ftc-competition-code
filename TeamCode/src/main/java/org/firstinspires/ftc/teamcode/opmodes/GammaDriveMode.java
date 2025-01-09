package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
@TeleOp
public class GammaDriveMode extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        topLeftMotor = hardwareMap.get(DcMotor.class,"leftFront");
        topRightMotor = hardwareMap.get(DcMotor.class,"rightFront");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"leftBack");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"rightBack");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        double y = -gamepad1.left_stick_y; // Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx2 = gamepad1.right_stick_x;

        topLeftMotor.setPower(y+ x + rx2);
        bottomLeftMotor.setPower(y - x + rx2);
        topRightMotor.setPower(y - x - rx2);
        bottomRightMotor.setPower(y + x - rx2);
    }
}
