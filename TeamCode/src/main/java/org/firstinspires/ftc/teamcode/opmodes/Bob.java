package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class Bob extends OpMode{

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;
    DcMotorEx side;
    DcMotorEx leftEx;
    DcMotorEx rightEx;
    int driving;
    double motorPower;
    double motorTurn;

    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
        side = hardwareMap.get(DcMotorEx.class, "side_ex");
        leftEx = hardwareMap.get(DcMotorEx.class, "left_ex");
        rightEx = hardwareMap.get(DcMotorEx.class, "right_ex");
        driving = 1;
    }

    @Override
    public void loop() {

        telemetry.addData("Drivemode: ", driving);

        if (gamepad1.b) {
            if (driving == 1) {
                driving = 2;
            }
            else {
                driving = 1;
            }
        }

        if (driving == 1) {
            motorPower = -gamepad1.left_stick_y * (1 - gamepad1.right_trigger);
            motorTurn = -gamepad1.right_stick_x * (1 - gamepad1.right_trigger);
            leftFront.setDirection(DcMotor.Direction.FORWARD);
            rightBack.setDirection(DcMotor.Direction.FORWARD);
            leftBack.setDirection(DcMotor.Direction.FORWARD);
            rightFront.setDirection(DcMotor.Direction.FORWARD);
            leftFront.setPower(-motorPower + motorTurn);
            leftBack.setPower(-motorPower + motorTurn);
            rightFront.setPower(motorPower + motorTurn);
            rightBack.setPower(motorPower + motorTurn);
        }
        else {
            if (gamepad1.dpad_up) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                motorPower = gamepad1.right_trigger;
           }
            else if (gamepad1.dpad_down) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                motorPower = -gamepad1.right_trigger;
            }
            else if (gamepad1.dpad_left) {
                leftFront.setDirection(DcMotor.Direction.REVERSE);
                rightBack.setDirection(DcMotor.Direction.REVERSE);
                leftBack.setDirection(DcMotor.Direction.FORWARD);
                rightFront.setDirection(DcMotor.Direction.FORWARD);
                motorPower = gamepad1.right_trigger;
            }
            else if (gamepad1.dpad_right) {
                leftFront.setDirection(DcMotor.Direction.FORWARD);
                rightBack.setDirection(DcMotor.Direction.FORWARD);
                leftBack.setDirection(DcMotor.Direction.REVERSE);
                rightFront.setDirection(DcMotor.Direction.REVERSE);
                motorPower = gamepad1.right_trigger;
            }
            else {
                motorPower =0;
            }
            leftFront.setPower(-motorPower);
            leftBack.setPower(-motorPower);
            rightFront.setPower(motorPower);
            rightBack.setPower(motorPower);
        }
    }
}
