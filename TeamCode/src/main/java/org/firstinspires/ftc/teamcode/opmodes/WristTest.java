package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class WristTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Servo SomethingServo;
        SomethingServo = hardwareMap.get(Servo.class, "CHANGETHISORURSTUPID");
        if(gamepad1.right_bumper){
            SomethingServo.setPosition(1);

        }
        else if (gamepad1.left_bumper){
            SomethingServo.setPosition(-1);
        }
        else if (!(gamepad1.left_bumper && gamepad1.right_bumper)){
            SomethingServo.setPosition(0);

    }

}
