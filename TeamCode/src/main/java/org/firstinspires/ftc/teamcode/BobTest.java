package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;


@TeleOp
public class BobTest extends OpMode {

    Servo leftUppy;
    double uppiness;

    @Override
    public void init() {
        leftUppy = hardwareMap.get(Servo.class, "left_uppy");
        uppiness = 0;
    }

    @Override
    public void loop () {
        if (gamepad1.a) {
            uppiness += 0.01;
        }
        if (gamepad1.b) {
            uppiness -= 0.01;
        }
        leftUppy.setPosition(uppiness);
    }
}
