package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class ClawToBasket extends LinearOpMode {
    private PIDController2 BasketPIDA;
    private PIDController2 BasketPIDE;
    double BasketKi=0;
    double BasketKp=0;
    double BasketKd=0;
    DcMotor Arm;
    DcMotor Elbow;
    DcMotor Viper;

    @Override
    public void runOpMode() throws InterruptedException {
        Toggle BasketToggle;
        BasketToggle = new Toggle();
        BasketToggle.update(gamepad1.y);

        double BasketReference = 0;


        PIDController2 BasketPID;
        BasketPID = new PIDController2(BasketReference, BasketKi, BasketKp, BasketKd);
        //BasketPID.update(0);
        if(BasketToggle.state) {
            BasketPIDA = new PIDController2(BasketReference, BasketKi, BasketKp, BasketKd);
            BasketPIDE = new PIDController2(BasketReference, BasketKi, BasketKp, BasketKd);

            //Arm.setPower(BasketPIDA.update(Arm.getCurrentPosition()));
            //Elbow.setPower(BasketPIDA.update(Elbow.getCurrentPosition()));
            //Viper.setPower(BasketPIDA.update(Viper.getCurrentPosition()));

        }

    }
}
