package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class WristTest extends LinearOpMode {

    public void runOpmode() throws InterruptedException{

        waitForStart();
        Servo wrist;
        double servoPos = 0.5;
        wrist =hardwareMap.get(Servo.class, "wrist");
        if (gamepad2.left_stick_x > 0.5){
            wrist.setPosition(servoPos);
            servoPos = servoPos = 0.1;

        }



    }


}
