package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@Config
@TeleOp
public class WristTest extends LinearOpMode {

    public void runOpmode() throws InterruptedException{

        waitForStart();
        Servo wrist;
        double servoPos = 0.5;
        wrist =hardwareMap.get(Servo.class, "wrist");

        while (opModeIsActive()){
        if (gamepad2.left_bumper){
            wrist.setPosition(servoPos);
            servoPos = servoPos - .1;

        }
        if (gamepad2.right_bumper){
            wrist.setPosition(servoPos);
            servoPos = servoPos + .1;

        }



    }


}
