package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class Bob extends OpMode{

    DcMotorEx leftFront;
    DcMotorEx leftBack;
    DcMotorEx rightFront;
    DcMotorEx rightBack;

    @Override
    public void init(){
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");
    }

    @Override
    public void loop() {
        double motorPower = -gamepad1.right_stick_y;
        double motorTurn = -2 * gamepad1.right_stick_x;
        leftFront.setPower(-motorPower + motorTurn);
        leftBack.setPower(-motorPower + motorTurn);
        rightFront.setPower(motorPower + motorTurn);
        rightBack.setPower(motorPower + motorTurn);
    }
}
