package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;
import org.firstinspires.ftc.teamcode.util.debouncer;

@Config
@TeleOp
public class fullArmTest extends LinearOpMode {
    private PIDController2 PIDE;
    private PIDController2 PIDA;
    private PIDController2 BasketPIDE;
    double eKi = 0;
    double eKp = 0.005;
    double eKd = 0;
    double power = 0;
    double aKi = 0;
    double aKp = 0.005;
    double aKd = 0;
    DcMotor Arm;
    DcMotor elbow;
    DcMotor viper;
    double eReference = 0;
    double aReference= 0;
    debouncer ebouncedup = new debouncer();
    debouncer ebounceddown =new debouncer();
    debouncer abouncedup = new debouncer();
    debouncer abounceddown =new debouncer();

    public void runOpMode() throws InterruptedException {

        waitForStart();
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        PIDE = new PIDController2(eReference, eKi, eKp, eKd);
        PIDA = new PIDController2(aReference, aKi, aKp, aKd);

        waitForStart();
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        while (opModeIsActive()) {
//elbow
            if (ebouncedup.update(gamepad2.dpad_left) ){
                eReference = eReference + 5;
                PIDE.reference = eReference;

            } else if (ebounceddown.update(gamepad2.dpad_right)){
                eReference = eReference - 5;
                PIDE.reference = eReference;
            }

            if(eReference >= 630){
                eReference = 630;
            }
            elbow.setPower(PIDE.update(elbow.getCurrentPosition()));
            //arm

            if (abouncedup.update(gamepad2.dpad_up) ){
                aReference = aReference + 5;
                PIDA.reference = aReference;

            } else if (abounceddown.update(gamepad2.dpad_down)){
                aReference = aReference - 5;
                PIDA.reference = aReference;
            }
            Arm.setPower(PIDA.update(Arm.getCurrentPosition()));

            //slide

           /* if(gamepad2.left_stick_y > 0 || gamepad2.left_stick_y < 0){
                viper.setPower(0.5*gamepad2.left_stick_y);
            }
            else{
                viper.setPower(0);
            }
*/
            telemetry.addData("EReference", eReference);
            //telemetry.addData("up", gamepad2.dpad_up);
            //telemetry.addData("down?", gamepad2.dpad_down);
           telemetry.addData("AReference", aReference);
            telemetry.addData("Epos", elbow.getCurrentPosition());
            telemetry.addData("APos", Arm.getCurrentPosition());
            telemetry.update();




        }
    }

}


