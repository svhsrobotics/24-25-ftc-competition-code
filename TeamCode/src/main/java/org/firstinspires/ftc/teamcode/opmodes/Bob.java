package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Bob extends OpMode{

    @Override
    public void init() {

    }

    @Override
    public void loop(){
    telemetry.addData("LABEL", gamepad1.left_stick_x);
}
}
