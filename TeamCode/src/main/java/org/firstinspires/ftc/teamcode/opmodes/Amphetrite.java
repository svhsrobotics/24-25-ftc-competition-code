package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class Amphetrite extends LinearOpMode{

    @Override
    public void runOpMode() throws InterruptedException{
         final DcMotor outSlide = hardwareMap.get(DcMotor.class, "out");
         final DcMotor upSlide = hardwareMap.get(DcMotor.class, "notheight");
         final Servo outServo = hardwareMap.get(Servo.class, "outClawServo");
         final Servo outClawRotationServo = hardwareMap.get(Servo.class, "outClawRotationServo");
         final Servo upViperSlideArm = hardwareMap.get(Servo.class, "arm");
         final Servo upClaw = hardwareMap.get(Servo.class, "upclaw");
         final DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
         final DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
         final DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
         final DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
         final PIDController2 PidV = new PIDController2(1.2*0.02/1.28, 0.02*0.6, 0.075*0.02*1.28, 1);
         //TODO: TUNE THE FREAKING LOOP FOR THE "OUT" SLIDE
         final PIDController2 PidH = new PIDController2(0, 0.01, 0,1);
         final Toggle slidetToggle = new Toggle();
    double outReference = 0;
    double upReference = 0;

    outSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    outSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    outSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    //
    upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    upSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    while(opModeIsActive()){
       slidetToggle.update(gamepad1.left_stick_button);
       //out slide
        if(gamepad1.left_trigger != 0 && !slidetToggle.state){
            outReference = outReference - 10 * (double) gamepad1.left_trigger;
        }
        else if(gamepad1.right_trigger != 0 && !slidetToggle.state){
            outReference = outReference + 10* (double) gamepad1.right_trigger;
        }
        if (outReference >= 4890){
            outReference = 4890;}
        if(outReference <= 0){
            outReference = 0;}
        if(upReference <=0){
            upReference =0;
        }
        outSlide.setPower(PidV.usePIDLoop(outSlide.getCurrentPosition(), outReference));
        //up slide
        if(gamepad1.left_trigger != 0 && slidetToggle.state){
            upReference = upReference - 10 * (double) gamepad1.left_trigger;
        }
        else if(gamepad1.right_trigger != 0 && slidetToggle.state){
            upReference = upReference + 10* (double) gamepad1.right_trigger;
        }
        if (upReference >= 3000){
            upReference = 3000;}
        upSlide.setPower(PidH.usePIDLoop(upSlide.getCurrentPosition(), upReference));
        //out claw stuff
        outClawRotationServo.setPosition(1);

        if(gamepad1.left_bumper){
           outServo.setPosition(1);
        }
        else if(gamepad1.right_bumper){
            outServo.setPosition(0);
        }
        //drive
        double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        leftFrontMotor.setPower(y + x + rx);
        leftBackMotor.setPower(y - x + rx);
        rightFrontMotor.setPower(y - x - rx);
        rightBackMotor.setPower(y + x - rx);
        //telemetry :P
        telemetry.addData("out reference", outReference);
        telemetry.addData("up reference", upReference);
        telemetry.addData("left trigger", gamepad1.left_trigger);
        telemetry.addData("right trigger", gamepad1.right_trigger);
        telemetry.update();


    }

    }

}