package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.PIDController2;

@TeleOp
@Config
public class ViperSlideTest extends LinearOpMode {



   private DcMotor upSlide;
   private Servo servo;
    FtcDashboard dashboard = FtcDashboard.getInstance();
    Telemetry dashboardTelemetry = dashboard.getTelemetry();
    @Override
    public void runOpMode() throws InterruptedException {

                //TODO: make it so that this code only triggers after passthrough so you can use the triggers for both directions

       upSlide = hardwareMap.get(DcMotor.class, "notheight");
       servo = hardwareMap.get(Servo.class, "arm");
        double viperReference = 2000;
        upSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        PIDController2 PIDV = new PIDController2(0, 0.01, 0,1);
        //ku:0.023, tu:3.31,
        //0.6Ku
        //
        //1.2Ku/Tu
        //
        //0.075KuTu
        waitForStart();
        while (opModeIsActive()) {
            servo.setPosition(0.5);


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
            double power = PIDV.usePIDLoop(upSlide.getCurrentPosition(), viperReference);
            telemetry.addData("realpower",PIDV.usePIDLoop(upSlide.getCurrentPosition(), viperReference));
            /*if (power >0.7){
                power=0.7;
            }*/
            upSlide.setPower(power);



            dashboardTelemetry.addData( "pod",  upSlide.getCurrentPosition());
            dashboardTelemetry.addData("reference", viperReference);
            dashboardTelemetry.addData("w/ int", (int) ( 20*gamepad1.left_trigger));
            dashboardTelemetry.addData("non int",20 * gamepad1.left_trigger);
dashboardTelemetry.addData("power", power);
            dashboardTelemetry.update();



        }
    }
}