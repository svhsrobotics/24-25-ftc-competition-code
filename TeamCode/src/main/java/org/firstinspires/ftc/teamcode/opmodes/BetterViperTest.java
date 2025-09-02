package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
@Config
@TeleOp
public class BetterViperTest extends LinearOpMode {

DcMotorEx viperslide;
    @Override
    public void runOpMode() throws InterruptedException {
        viperslide = hardwareMap.get(DcMotorEx.class, "spinny");
        waitForStart();
        while (opModeIsActive()&&!isStopRequested()) {
            viperslide.setTargetPosition(-100);
            viperslide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viperslide.setPower(.3);
        }
    }
}
