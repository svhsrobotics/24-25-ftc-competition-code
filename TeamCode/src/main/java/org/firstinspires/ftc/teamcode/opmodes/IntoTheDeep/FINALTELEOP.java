package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import android.hardware.display.VirtualDisplayConfig;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.util.Debouncer;

@TeleOp
public class FINALTELEOP extends LinearOpMode{
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotorEx intake;
    private Servo gateServo;
    private Servo gateServo2;
    private VoltageSensor  voltSensor;
    private RevBlinkinLedDriver frontLights;
    private RevBlinkinLedDriver rearLights;


    private boolean debounce;
    private boolean isthethingthething;
    double totalCurrent = 0;
 int denominator = 0;
 double averageCurrent = 0;


    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch2=hardwareMap.get(DcMotor.class, "launch2");
        launch=hardwareMap.get(DcMotor.class, "launch1");
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");
        frontLights = hardwareMap.get(RevBlinkinLedDriver.class, "frontLights");
        rearLights = hardwareMap.get(RevBlinkinLedDriver.class, "rearLights");

gateServo2.setDirection(Servo.Direction.REVERSE);
        debounce=true;
        isthethingthething=false;
        Debouncer debouncingOnDeesNuts = new Debouncer();
        Debouncer debouncer2 = new Debouncer();
        double gatePos = 0;
        double launchpower = 0;
        System.out.println("set gatePos to 0");

        while(opModeInInit()){
            launchpower=0.9;
            frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
            rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLACK);
        }
        waitForStart();
        while (opModeIsActive()) {
            //todo; maybe make negative

            System.out.println("gatepos: " + gatePos);
            System.out.println("servo 1 pos: " + gateServo.getPosition());
            System.out.println("servo 2 pos: " + gateServo2.getPosition());


            right.setPower((gamepad1.right_stick_x + gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x - gamepad1.left_stick_y));
//-1 on servo 2
            if (gamepad1.dpad_up) { //opens da gate
                gateServo.setPosition(0.48); //i am a silly guy
                gateServo2.setPosition(0.48);
            } else if (gamepad1.dpad_down) { //close
                gateServo.setPosition(0.02);
                gateServo2.setPosition(0.02);
            }

            if (gamepad1.dpad_right) {
                launch.setPower(launchpower);
                launch2.setPower(launchpower);


            } else if (gamepad1.dpad_left) {
                launch.setPower(0);
                launch2.setPower(0);

            }

            if (debouncingOnDeesNuts.update(gamepad2.dpad_up)) {
                launchpower += .01;
            } else if (debouncer2.update(gamepad2.dpad_down)) {
                launchpower -= .01;
            }

            telemetry.addData("shoot power", launchpower);
            telemetry.addData("servo1Pos: ", gateServo.getPosition());
            telemetry.addData("servo2Pos", gateServo2.getPosition());
            telemetry.addData("average milliamp",  averageCurrent);
            telemetry.update();

            if(gamepad1.b){
                intake.setPower(1);
                gateServo.setPosition(.02);
                gateServo2.setPosition(.02);
                launch.setPower(0);
                launch2.setPower(0);
            }

            if(gamepad1.right_bumper){
                intake.setPower(0);
            }

            if(gamepad1.a){
                intake.setPower(0);
                if(launchpower == launch.getPower()) {
                    gateServo.setPosition(0.48);
                    gateServo2.setPosition(0.48);
                } else if (launchpower != launch.getPower()) {
                    launch.setPower(launchpower);
                    launch2.setPower(launchpower);
                    right.setPower(0);
                    left.setPower(0);
                    intake.setPower(0.2);
                    sleep(6000);
                    gateServo.setPosition(0.48);
                    gateServo2.setPosition(0.48);
                    sleep(100);
                    intake.setPower(0);
                }
            }

            if(gamepad2.y){
                intake.setPower(1);
            }

            if(gamepad2.x){
                intake.setPower(0);
            }
            if(gamepad2.a){
                intake.setPower(-1);
            }


           // if ((intake.getCurrent(CurrentUnit.MILLIAMPS) > averageCurrent * 3) ){
            //    frontLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
            //    rearLights.setPattern(RevBlinkinLedDriver.BlinkinPattern.ORANGE);
           // } else if(intake.getCurrent(CurrentUnit.MILLIAMPS) > 5) {
            //    totalCurrent += intake.getCurrent(CurrentUnit.MILLIAMPS);
            //    denominator += 1;
              //  averageCurrent = totalCurrent/denominator;
            //}

           // if (voltSensor.getVoltage() < 11.5) {
            //    telemetry.addLine("YOUR VOLTAGE IS LOW");
             //   telemetry.update();
           // }
        }


    }
}
