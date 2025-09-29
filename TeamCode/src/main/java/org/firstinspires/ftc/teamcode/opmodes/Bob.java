package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.ArrayList;

@TeleOp
public class Bob extends OpMode{

    ArrayList<String> bobStuff = new ArrayList<String>();


    @Override
    public void init() {
        bobStuff.add("Cool");
        bobStuff.add("Hi");
        bobStuff.add("This game is fum!");
        bobStuff.set(2, "THIS GAME IS WIGGED!!!!!!!!!!");
    }

    @Override
    public void loop(){
    telemetry.addData("LABEL", gamepad1.left_stick_x);
}
}
