package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp
public class WristTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        Servo SomethingServo;
        SomethingServo = hardwareMap.get(Servo.class, "CHANGETHISORURSTUPID");


        while (opModeIsActive()) {
            if(gamepad1.right_bumper){
                SomethingServo.setPosition(0.1);
            }
            else {
                SomethingServo.setPosition(0);

            }

            telemetry.addData("servo pos", SomethingServo.getPosition());
            telemetry.addData("bumper?", gamepad1.right_bumper);
            telemetry.addData("bumper2?", gamepad1.left_bumper);
            telemetry.update();
            //:P
    }

    }
}



