package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Debouncer;

public class PREDICTION extends LinearOpMode {
    public void runOpMode() throws InterruptedException {
         final DcMotor right = hardwareMap.get(DcMotor.class, "right");
         right.setDirection(DcMotorSimple.Direction.REVERSE);
        final DcMotor left = hardwareMap.get(DcMotor.class, "left");

        final Servo turret = hardwareMap.get(Servo.class, "turret");
        final Servo angle = hardwareMap.get(Servo.class, "angle");
        double turretAngle = 0;

         final Debouncer craigUp = new Debouncer();
         final Debouncer craigDown = new Debouncer();
        waitForStart();




        /*
        | \
        |  \
        |___\
         */
        while (opModeIsActive()) {
            left.setPower(-gamepad1.left_stick_y);
            right.setPower(-gamepad1.right_stick_y);

          //todo:  turretAngle = Math.tan(x/y)






        }
    }
}



