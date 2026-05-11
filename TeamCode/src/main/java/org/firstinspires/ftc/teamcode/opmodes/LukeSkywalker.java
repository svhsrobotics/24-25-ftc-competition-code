package org.firstinspires.ftc.teamcode.opmodes;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class LukeSkywalker extends OpMode {

    DcMotor leg;
    DcMotor legDos;

    @Override
    public void init() {
        leg = hardwareMap.get(DcMotor.class, "walker");
        legDos = hardwareMap.get(DcMotor.class, "limb");

        leg.setDirection(DcMotor.Direction.FORWARD);
        legDos.setDirection(DcMotor.Direction.REVERSE);
    }

    @Override
    public void loop() {
        leg.setPower(-0.4 * gamepad1.left_stick_y);
        legDos.setPower(-0.4 * gamepad1.left_stick_y);
    }
}
