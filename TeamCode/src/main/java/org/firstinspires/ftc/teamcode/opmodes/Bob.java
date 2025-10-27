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
        leftShoot = hardwareMap.get(DcMotor.class, "leftShoot");
        rightShoot = hardwareMap.get(DcMotor.class, "rightShoot");
        intake = hardwareMap.get(DcMotor.class, "intake");
        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
        leftShoot.setDirection(DcMotor.Direction.FORWARD);
        rightShoot.setDirection(DcMotor.Direction.REVERSE);
        intake.setDirection(DcMotor.Direction.FORWARD);
    }

    @Override
    public void loop () {

        //this driving is broken
        rx = gamepad1.left_stick_y;
        y = -gamepad1.left_stick_x;
        x = -gamepad1.right_stick_x;
        /*replace below gamepad1 with gamepad2:
        shoot = -gamepad2.right_stick_y;     */
        if (gamepad1.a) {
            shoot = -0.85;
        }
        else {
            shoot = 0;
        }

        leftFront.setPower(0.5 * (y + x + rx));
        leftBack.setPower(0.5 * (y - x + rx));
        rightFront.setPower(0.5 * (y - x - rx));
        rightBack.setPower(0.5 * (y + x - rx));
        leftShoot.setPower(shoot);
        rightShoot.setPower(shoot);

        /*replace below gamepad1 with gamepad2:
        if (gamepad2.a)...    */
        intake.setPower(gamepad1.right_trigger * 0.75);
        intake.setPower(-gamepad1.left_trigger * 0.75);
    }

}
