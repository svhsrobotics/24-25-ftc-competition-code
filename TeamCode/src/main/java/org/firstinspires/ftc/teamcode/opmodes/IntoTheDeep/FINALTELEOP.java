package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import android.hardware.display.VirtualDisplayConfig;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class FINALTELEOP extends LinearOpMode{
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch2;
    private DcMotor launch;
    private DcMotor intake;
    private Servo gateServo;
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
        gateServo.setDirection(Servo.Direction.REVERSE);
        debounce=true;
        isthethingthething=false;
        waitForStart();
        while (opModeIsActive()) {
            launch2.setPower(gamepad1.left_trigger);
            launch.setPower(gamepad1.left_trigger);
            intake.setPower(gamepad1.right_trigger);
            right.setPower((gamepad1.right_stick_x+ gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x -gamepad1.left_stick_y));
            if (gamepad1.b) {
                if (debounce) {
                    debounce=false;
                    isthethingthething = !isthethingthething;
                    if (isthethingthething) {
                        isthethingthething = !isthethingthething;
                        gateServo.setPosition(0.9);
                    } else {
                        gateServo.setPosition(0);
                    }
                }
            }
        }

        if (!gamepad1.b) {
            debounce = true;
        }



    }
}
