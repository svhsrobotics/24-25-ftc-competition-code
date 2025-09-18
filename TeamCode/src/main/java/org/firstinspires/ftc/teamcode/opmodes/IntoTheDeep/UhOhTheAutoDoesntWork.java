package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class UhOhTheAutoDoesntWork extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
         DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
         DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
         DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
         DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//i did all the work

        waitForStart();

      leftBackMotor.setPower(-1);
      leftFrontMotor.setPower(-1);
      rightBackMotor.setPower(-1);
      leftFrontMotor.setPower(-1);
      sleep(2000); //todo maybe adjust
        }
    }

