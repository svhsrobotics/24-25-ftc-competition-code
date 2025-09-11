package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDController2;

@TeleOp
@Config
public class OutViperSlideTest extends LinearOpMode {

   private DcMotor outSlide;
    @Override
    public void runOpMode() throws InterruptedException {

                //TODO: make it so that this code only triggers after passthrough so you can use the triggers for both directions

       outSlide = hardwareMap.get(DcMotor.class, "out");
        double viperReference = 2000;
        outSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        outSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        PIDController2 PIDV = new PIDController2(0, 0.55, 0, 1);
        //ku: 1
        //tu: 3.11
        //0.6Ku
        //
        //1.2Ku/Tu
        //
        //0.075KuTu
        waitForStart();
        while (opModeIsActive()) {


            if (viperReference >= 3000){
                viperReference = 3000;}


            if ( (double) gamepad1.left_trigger != 0)
                viperReference = viperReference + (int) ( 20*gamepad1.left_trigger);
            else if ( (double) gamepad1.right_trigger != 0) {
                viperReference = viperReference - (int) (20*gamepad1.right_trigger);
            }


            if(viperReference<=0){
                viperReference=0;
            }
            double power = PIDV.usePIDLoop(outSlide.getCurrentPosition(), viperReference);
            telemetry.addData("realpower",PIDV.usePIDLoop(outSlide.getCurrentPosition(), viperReference));

            outSlide.setPower(power);

            telemetry.addData( "pod",  outSlide.getCurrentPosition());
            telemetry.addData("reference", viperReference);
            telemetry.addData("left w/ int", (int) ( 20*gamepad1.left_trigger));
            telemetry.addData("left non int",20 * gamepad1.left_trigger);
            telemetry.addData("right w/ int", (int) ( 20*gamepad1.right_trigger));
            telemetry.addData("right non int",20 * gamepad1.right_trigger);
telemetry.addData("power", power);
            telemetry.update();



        }
    }
}