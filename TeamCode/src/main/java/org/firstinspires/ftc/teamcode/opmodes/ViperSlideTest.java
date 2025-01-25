package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDController2;

@TeleOp
@Config
public class ViperSlideTest extends LinearOpMode {

   private DcMotor spinny;
    @Override
    public void runOpMode() throws InterruptedException {

                //TODO: make it so that this code only triggers after passthrough so you can use the triggers for both directions

       spinny = hardwareMap.get(DcMotor.class, "notheight");
        double viperReference = 0;
        spinny.setDirection(DcMotorSimple.Direction.REVERSE);

        spinny.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spinny.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //it breaks sometimes idk why
        PIDController2 PIDV = new PIDController2(0, 0.01, 0, 1);
        waitForStart();
        while (opModeIsActive()) {


            if (viperReference >= 4890){
                viperReference = 4890;}


            if ( (double) gamepad1.left_trigger != 0)
                viperReference = viperReference + (int) ( 20*gamepad1.left_trigger);
            else if ( (double) gamepad1.right_trigger != 0) {
                viperReference = viperReference - (int) (20*gamepad1.right_trigger);
            }


            if(viperReference<=0){
                viperReference=0;
            }
            double power = PIDV.usePIDLoop(spinny.getCurrentPosition(), viperReference);
            telemetry.addData("realpower",PIDV.usePIDLoop(spinny.getCurrentPosition(), viperReference));
            /*if (power >0.7){
                power=0.7;
            }*/
            spinny.setPower(power);


            telemetry.addData( "pod",  spinny.getCurrentPosition());
            telemetry.addData("reference", viperReference);
            telemetry.addData("w/ int", (int) ( 20*gamepad1.left_trigger));
            telemetry.addData("non int",20 * gamepad1.left_trigger);
telemetry.addData("power", power);
            telemetry.update();



        }
    }
}