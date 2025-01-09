package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Debouncer;

@Config
@TeleOp
public class elbowtest extends LinearOpMode {
    private PIDController2 PIDE;
    private PIDController2 BasketPIDE;
    double eKi = 0;
    double eKp = 0.005;
    double eKd = 0;
    double power = 0;
    DcMotor Arm;
    DcMotor elbow;

    DcMotor Viper;
    double eReference = 0;

    Debouncer bouncedup = new Debouncer();
    Debouncer bounceddown =new Debouncer();


    @Override
    public void runOpMode() throws InterruptedException {
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        PIDE = new PIDController2(eReference, eKi, eKp, eKd);

        waitForStart();
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {

                if (bouncedup.update(gamepad2.dpad_up) ){
                    eReference = eReference + 5;
                    PIDE.reference = eReference;

                } else if (bounceddown.update(gamepad2.dpad_down)){
                    eReference = eReference - 5;
                    PIDE.reference = eReference;
                }

                if(eReference >= 630){
                    eReference= 630;
                }



                        //power = PIDE.update(elbow.getCurrentPosition());
                elbow.setPower(power);
                        telemetry.addData("reference", eReference);
                        telemetry.addData("up", gamepad2.dpad_up);
                        telemetry.addData("down?", gamepad2.dpad_down);
                        telemetry.addData("power", power);
                        telemetry.addData("pos", elbow.getCurrentPosition());
                        telemetry.update();




            }
        }

    }

