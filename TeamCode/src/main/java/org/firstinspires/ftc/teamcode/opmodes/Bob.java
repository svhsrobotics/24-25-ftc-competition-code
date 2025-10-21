package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class Bob extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    DcMotor leftShoot;
    DcMotor rightShoot;
    DcMotor intake;
    Servo leftUppy;
    Servo rightUppy;
    double motorPower;
    double motorTurn;
    double shoot;
    double y;
    double x;
    double rx;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "left_front");
        leftBack = hardwareMap.get(DcMotor.class, "left_back");
        rightFront = hardwareMap.get(DcMotor.class, "right_front");
        rightBack = hardwareMap.get(DcMotor.class, "right_back");
        leftShoot = hardwareMap.get(DcMotor.class, "left_shoot");
        rightShoot = hardwareMap.get(DcMotor.class, "right_shoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftUppy = hardwareMap.get(Servo.class, "left_uppy");
        rightUppy = hardwareMap.get(Servo.class, "right_uppy");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftShoot.setDirection(DcMotor.Direction.REVERSE);
        rightShoot.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop () {
        telemetry.addData("PI: ", Math.PI);

        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;
        rx = gamepad1.right_stick_x;
        shoot = -gamepad2.right_stick_y;
        leftFront.setPower(y + x + rx);
        leftBack.setPower(y - x + rx);
        rightFront.setPower(y - x - rx);
        rightBack.setPower(y + x - rx);
        leftShoot.setPower(shoot);
        rightShoot.setPower(shoot);

        if (gamepad2.a) {
                intake.setPower(0.6);
            }
        else{
                intake.setPower(0);
            }
        }
}
