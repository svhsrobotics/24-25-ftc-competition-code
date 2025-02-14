package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class NoRoadrunner extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
         DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
         DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
         DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
         DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        final Servo upViperSlideArm = hardwareMap.get(Servo.class, "arm");
        upViperSlideArm.setDirection(Servo.Direction.REVERSE);


        waitForStart();

        upViperSlideArm.setPosition(0.08);




                leftBackMotor.setPower(1);
                leftFrontMotor.setPower(1);
                rightBackMotor.setPower(1);
                rightFrontMotor.setPower(1);
                sleep(200);


            leftBackMotor.setPower(0);
            leftFrontMotor.setPower(0);
            rightBackMotor.setPower(0);
            rightFrontMotor.setPower(0);
            upViperSlideArm.setPosition(0.08);
            sleep(1000);
        leftBackMotor.setPower(1);
        leftFrontMotor.setPower(1);
        rightBackMotor.setPower(1);
        rightFrontMotor.setPower(1);
        sleep(100);
        }
    }

