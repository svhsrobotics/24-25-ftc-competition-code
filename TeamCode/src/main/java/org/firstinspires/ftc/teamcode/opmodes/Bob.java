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
    int driving;
    double uppiness;
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
        driving = 1;
        uppiness = 0;
    }

    @Override
    public void loop() {

        telemetry.addData("Drive mode: ", driving);


        //Drive mode Switching
        if (gamepad1.b) {
            if (driving == 1) {
                driving = 2;
            } else {
                driving = 1;
            }
        }

        /* Driving       |
                         |
                         v
         */
        if (driving == 1) {
            //Joystick driving without strafe
            motorPower = -gamepad1.left_stick_y * (1 - gamepad1.right_trigger);
            motorTurn = -gamepad1.right_stick_x * (1 - gamepad1.right_trigger);
            leftFront.setPower(-motorPower + motorTurn);
            leftBack.setPower(-motorPower + motorTurn);
            rightFront.setPower(motorPower + motorTurn);
            rightBack.setPower(motorPower + motorTurn);
        } else {
            //D-pad & strafe
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;
            leftFront.setPower(y + x + rx);
            leftBack.setPower(y - x + rx);
            rightFront.setPower(y - x - rx);
            rightBack.setPower(y + x - rx);

            //aiming
            if (gamepad2.a) {
                uppiness = uppiness += 0.01;
            }
            if (gamepad2.b) {
                uppiness = uppiness -= 0.01;
            }
            leftUppy.setPosition(uppiness);
            rightUppy.setPosition(uppiness);

            shoot = -gamepad2.right_stick_y + 1;
        }
    }
}
