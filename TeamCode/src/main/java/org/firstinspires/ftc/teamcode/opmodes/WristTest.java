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
        Servo wrist;
        double servoPos;

        //
        waitForStart();
        //

        servoPos = 0.5;
        wrist = hardwareMap.get(Servo.class, "blackCable");
        wrist.setPosition(servoPos);

        while (opModeIsActive()) {
            if (gamepad2.left_bumper) {
                wrist.setPosition(servoPos);
                servoPos = 0;

            }
            if (gamepad2.right_bumper){
                wrist.setPosition(servoPos);
                servoPos = 1;

            }
                /*if(servoPos>=1){
                    servoPos = 1;
                }
                else if (servoPos<=0){
                    servoPos=0;
                }*/

                telemetry.addData("servoPos", servoPos);
                telemetry.addLine("jjtech");
                telemetry.update();




        }


    }





}
