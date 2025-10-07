package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous
public class LinearBob  extends LinearOpMode {
    DcMotor leftDrive;
    DcMotor rightDrive;
    ElapsedTime never = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        leftDrive = hardwareMap.get(DcMotor.class, "left_motor");
        rightDrive = hardwareMap.get(DcMotor.class, "right_motor");

        waitForStart();

        leftDrive.setPower(1);
        rightDrive.setPower(-1);
        
    }
}
