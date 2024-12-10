package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class elbowtest extends LinearOpMode {
    private PIDController2 PIDE;
    private PIDController2 BasketPIDE;
    double eKi=0;
    double eKp=0;
    double eKd=0;
    DcMotor Arm;
    DcMotor Elbow;
    DcMotor Viper;
    double eReference = 0;
    Toggle eTog = new Toggle();


    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {
            Elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            eTog.update(gamepad1.start || gamepad2.start);


            if (gamepad2.dpad_up){
                eReference = eReference +5;

            }
            else if (gamepad2.dpad_down){
                eReference = eReference - 5;
            }
            PIDE = new PIDController2(eReference, 0, 0, 0);

            Elbow.setPower(PIDE.update(Elbow.getCurrentPosition()));


        }
    }

}
