package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import android.hardware.display.VirtualDisplayConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.util.Debouncer;

@TeleOp
public class FINALTELEOP extends LinearOpMode{
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotor intake;
    private Servo gateServo;
    private Servo gateServo2;
    private VoltageSensor  voltSensor;


    private boolean debounce;
    private boolean isthethingthething;
// one way to make sure youre code is not stolen, is to make sure no one can read it -truman


    @Override
    public void runOpMode() throws InterruptedException {
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch2=hardwareMap.get(DcMotor.class, "launch2");
        launch=hardwareMap.get(DcMotor.class, "launch1");
        launch.setDirection(DcMotorSimple.Direction.REVERSE);
        intake = hardwareMap.get(DcMotor.class, "intake");
        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");
        voltSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

gateServo2.setDirection(Servo.Direction.REVERSE);
        debounce=true;
        isthethingthething=false;
        Debouncer debouncingOnDeesNuts = new Debouncer();
        Debouncer debouncer2 = new Debouncer();
        double gatePos = 0;
        double launchpower = 0;
        System.out.println("set gatePos to 0");
        waitForStart();
        while (opModeIsActive()) {
            //todo; maybe make negative

System.out.println("gatepos: " + gatePos);
            System.out.println("servo 1 pos: " + gateServo.getPosition());
            System.out.println("servo 2 pos: " + gateServo2.getPosition());





            intake.setPower(gamepad1.right_trigger);
            right.setPower((gamepad1.right_stick_x+ gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x -gamepad1.left_stick_y));
//-1 on servo 2
           if(gamepad1.b){
                gateServo.setPosition(0.48); //i am a silly guy
                gateServo2.setPosition(.48);
            } else if(gamepad1.x){
                gateServo.setPosition(0.02);
                gateServo2.setPosition(0.02);
            }

           if(gamepad1.a){
               launch.setPower(launchpower);
               launch2.setPower(launchpower);


           }
           else if(gamepad1.y){
               launch.setPower(0);
               launch2.setPower(0);

           }

           if(debouncingOnDeesNuts.update(gamepad1.dpad_up)){
              launchpower += .01;
           }else if(debouncer2.update(gamepad1.dpad_down)){
               launchpower -= .01;
           }

           telemetry.addData("shoot power", launchpower);
           telemetry.addData("servo1Pos: ", gateServo.getPosition());
           telemetry.addData("servo22 pos", gateServo2.getPosition());
           telemetry.update();


        }

        if(gamepad1.left_trigger != 0){
            intake.setPower(gamepad1.left_trigger);
        } else if (gamepad1.right_trigger != 0){
            intake.setPower(-gamepad1.right_trigger);
        }

        if(voltSensor.getVoltage() < 11){
            telemetry.addLine("YOUR VOLTAGE IS LOW");
            telemetry.update();
        }






    }
}
