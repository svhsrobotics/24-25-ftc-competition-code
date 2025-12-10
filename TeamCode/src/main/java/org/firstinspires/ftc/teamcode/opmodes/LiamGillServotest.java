package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class LiamGillServotest extends LinearOpMode {

    private Servo gateServo;
    private Servo gateServo2;

    @Override
    public void runOpMode() throws InterruptedException {

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        gateServo2 = hardwareMap.get(Servo.class, "gateServo2");

        waitForStart();
        while (opModeIsActive()) {

            gateServo.setPosition(1);
            gateServo2.setPosition(1);
//test
        }
    }
}
