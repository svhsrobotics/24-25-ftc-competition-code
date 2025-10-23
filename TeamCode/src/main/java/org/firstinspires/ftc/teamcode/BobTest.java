package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class BobTest extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    double shoot;
    double y;
    double x;
    double rx;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");
        leftFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.REVERSE);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop () {

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        //backwards for LSY
        //leftFront.setPower(0.5 * (y + x + rx));
        leftBack.setPower(0.5 * (y - x + rx));
        //rightFront.setPower(0.5 * (y - x - rx));
        //rightBack.setPower(0.5 * (y + x - rx));
    }
}
