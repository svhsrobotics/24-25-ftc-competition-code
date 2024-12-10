package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class ClawToNeutral extends LinearOpMode {
    private PIDController2 NeutralPIDA;
    private PIDController2 NeutralPIDE;
    double NeutralKi=0;
    double NeutralKp=0;
    double NeutralKd=0;
    DcMotor Arm;
    DcMotor Elbow;
    DcMotor Viper;

    @Override
    public void runOpMode() throws InterruptedException {
        Toggle NeutralToggle;
        NeutralToggle = new Toggle();
        NeutralToggle.update(gamepad1.y);

        double NeutralReference = 0;


        PIDController2 BasketPID;
        BasketPID = new PIDController2(NeutralReference, NeutralKi, NeutralKp, NeutralKd);
        BasketPID.update(0);
        if(NeutralToggle.state) {
            NeutralPIDA = new PIDController2(NeutralReference, NeutralKi, NeutralKp, NeutralKd);
            NeutralPIDE = new PIDController2(NeutralReference, NeutralKi, NeutralKp, NeutralKd);

            Arm.setPower(NeutralPIDA.update(Arm.getCurrentPosition()));
            Elbow.setPower(NeutralPIDA.update(Elbow.getCurrentPosition()));
            Viper.setPower(NeutralPIDA.update(Viper.getCurrentPosition()));

        }

    }
}
