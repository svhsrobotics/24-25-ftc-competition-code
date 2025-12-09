package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Debouncer;

public class BetterTeleop extends LinearOpMode {
private Debouncer debounce = new Debouncer();

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor launch2 = hardwareMap.get(DcMotor.class, "launch2");
        final DcMotor launch = hardwareMap.get(DcMotor.class, "launch1");
       final DcMotor intake = hardwareMap.get(DcMotor.class, "intake");
        final Servo gateServo = hardwareMap.get(Servo.class, "gateServo");
        final DcMotor right = hardwareMap.get(DcMotor.class, "right");
        final DcMotor left = hardwareMap.get(DcMotor.class, "left");




        while(opModeIsActive()){
            launch2.setPower(gamepad1.left_trigger);
            launch.setPower(gamepad1.left_trigger);
            intake.setPower(gamepad1.right_trigger);
            right.setPower((gamepad1.right_stick_x+ gamepad1.left_stick_y));
            left.setPower((gamepad1.right_stick_x -gamepad1.left_stick_y));

            debounce.update(gamepad1.b);






        }
    }
}
