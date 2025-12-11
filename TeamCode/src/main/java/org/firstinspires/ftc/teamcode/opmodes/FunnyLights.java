package org.firstinspires.ftc.teamcode.opmodes;

import android.animation.RectEvaluator;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FunnyLights extends LinearOpMode {

    private RevBlinkinLedDriver frontLights;
    private RevBlinkinLedDriver rearLights;

    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        frontLights = hardwareMap.get(RevBlinkinLedDriver.class,"frontLights");
        rearLights = hardwareMap.get(RevBlinkinLedDriver.class,"rearLights");

        while(opModeIsActive()) {
            frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
            rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        }
    }
}