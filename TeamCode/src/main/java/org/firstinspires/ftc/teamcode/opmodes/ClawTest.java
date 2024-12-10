package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class ClawTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();
        Toggle tog = new Toggle();

        Servo SomethingServo;
        SomethingServo = hardwareMap.get(Servo.class, "CHANGETHISORURSTUPID");


        while (opModeIsActive()) {

            tog.update(gamepad2.right_bumper);

            if(tog.state){
                SomethingServo.setPosition(0);
            }

            else {
                SomethingServo.setPosition(1);

            }

            telemetry.addData("servo pos", SomethingServo.getPosition());
            telemetry.addData("bumper?", gamepad1.right_bumper);
            telemetry.addData("bumper2?", gamepad1.left_bumper);
            telemetry.update();
            //:P
    }

    }
}



