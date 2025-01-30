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
    private final DcMotor outSlide = hardwareMap.get(DcMotor.class, "out");
    private final DcMotor upSlide = hardwareMap.get(DcMotor.class, "notheight");
    private final Servo outServo = hardwareMap.get(Servo.class, "outClawServo");
    private final Servo outClawRotationServo = hardwareMap.get(Servo.class, "outClawRotationServo");
    private final Servo upViperSlideArm = hardwareMap.get(Servo.class, "arm");
    private final PIDController2 PidV = new PIDController2(1.2*0.02/1.28, 0.02*0.6, 0.075*0.02*1.28, 1);
    private final PIDController2 PidH = new PIDController2(0, 0.02, 0,1);
    private final Toggle slidetToggle = new Toggle();
    @Override
    public void runOpMode() throws InterruptedException{
    double outReference = 0;
    double upReference = 0;
    outSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    outSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    outSlide.setDirection(DcMotorSimple.Direction.REVERSE);
    //
    upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    upSlide.setDirection(DcMotorSimple.Direction.REVERSE);

    waitForStart();

    while(opModeIsActive()){
       slidetToggle.update(gamepad1.left_stick_button);
       //out slide
        if(gamepad1.left_trigger != 0 && !slidetToggle.state){
            outReference = outReference - 10 * (double) gamepad1.left_trigger;
        }
        else if(gamepad1.right_trigger != 0 && !slidetToggle.state){
            outReference = outReference - 10* (double) gamepad1.left_trigger;
        }
        if (outReference >= 4890){
            outReference = 4890;}
        outSlide.setPower(PidV.usePIDLoop(outSlide.getCurrentPosition(), outReference));
        //up slide
        if(gamepad1.left_trigger != 0 && slidetToggle.state){
            upReference = upReference - 10 * (double) gamepad1.left_trigger;
        }
        else if(gamepad1.right_trigger != 0 && slidetToggle.state){
            upReference = upReference - 10* (double) gamepad1.left_trigger;
        }
        if (upReference >= 3000){
            upReference = 3000;}
        upSlide.setPower(PidH.usePIDLoop(upSlide.getCurrentPosition(), upReference));


    }

    }

}