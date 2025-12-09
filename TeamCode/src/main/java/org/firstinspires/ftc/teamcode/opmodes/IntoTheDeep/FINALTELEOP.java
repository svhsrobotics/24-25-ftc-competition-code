package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import android.hardware.display.VirtualDisplayConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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

        debounce=true;
        isthethingthething=false;
        Debouncer bouncingOnDeesNuts = new Debouncer();
        double gatePos = 0;
        System.out.println("set gatePos to 0");
        waitForStart();
        while (opModeIsActive()) {
            //todo; maybe make negative

System.out.println("gatepos: " + gatePos);
            System.out.println("servo 1 pos: " + gateServo.getPosition());
            System.out.println("servo 2 pos: " + gateServo2.getPosition());




            launch2.setPower(gamepad1.left_trigger);
            launch.setPower(gamepad1.left_trigger);
            intake.setPower(gamepad1.right_trigger);
            right.setPower((gamepad1.right_stick_x+ gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x -gamepad1.left_stick_y));
//-1 on servo 2
            if(gamepad1.a){
                gateServo.setPosition(1); //i am a silly guy
                gateServo2.setPosition(-1);
            }
            else if(gamepad1.b){
                gateServo.setPosition(-.9); //i am a silly guy
                gateServo2.setPosition(1);
            }


        }



    }
}
