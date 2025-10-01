package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@Config
@TeleOp
public class testdticks extends LinearOpMode {


    private DcMotor viper;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()){
            viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



    }

    }}