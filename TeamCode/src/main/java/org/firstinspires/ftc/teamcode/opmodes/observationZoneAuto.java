package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous
public class observationZoneAuto extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private int count = 5;




    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");

    waitForStart();
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
for (int i=0; i<10000; i++) {
    leftFrontMotor.setPower(0.8);
    rightFrontMotor.setPower(0.8);
    leftBackMotor.setPower(0.8);
    rightBackMotor.setPower(0.8);
}
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);
                leftBackMotor.setPower(0);
                rightBackMotor.setPower(0);

    }
}



