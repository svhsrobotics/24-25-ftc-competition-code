package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import java.util.ArrayList;

@TeleOp
public class Bob extends OpMode{

    DcMotor leftDrive;
    DcMotor rightDrive;

    @Override
    public void init() {
        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");
    }

    @Override
    public void loop() {
        double motorPower = -gamepad1.right_stick_y;
        double motorTurn = gamepad1.right_stick_x;
        leftDrive.setPower(motorPower + motorTurn);
        rightDrive.setPower(-(motorPower - motorTurn));
    }
}
