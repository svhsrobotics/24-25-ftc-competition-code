package org.firstinspires.ftc.teamcode.opmodes;

import android.animation.RectEvaluator;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FunnyLights extends LinearOpMode {
    private RevBlinkinLedDriver lightstrip;
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        lightstrip = hardwareMap.get(RevBlinkinLedDriver.class,"lightstrip");




        while(opModeIsActive()) {
         lightstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
         sleep(10);
         lightstrip.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
         sleep(10);

        }
    }
}