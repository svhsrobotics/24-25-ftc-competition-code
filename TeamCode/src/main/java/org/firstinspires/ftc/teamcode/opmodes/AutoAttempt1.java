/*package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController2;

@Config
@Autonomous(name = "INeedToNameThisSomething", group = "Autonomous" )
public class AutoAttempt1 extends LinearOpMode {
    public class Lift {
        private DcMotor upSlide;

        public Lift(HardwareMap hardwareMap) {

            upSlide = hardwareMap.get(DcMotor.class, "notheight");
            upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            upSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public class highBasket implements Action {
            private PIDController2 PID = new PIDController2(1.2 * 0.02 / 1.28, 0.02 * 0.6, 0.075 * 0.02 * 1.28, 1);

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upSlide.setPower(PID.usePIDLoop(upSlide.getCurrentPosition(), 4000));
                if (upSlide.getCurrentPosition() == 4000) {
                    return false;
                } else {
                    return true;
                }


            }
            public Action goToHighBasket(){
                return new highBasket();
            }
        }
        public class goToZero implements  Action{
           public boolean run(@NonNull TelemetryPacket packet){

               if(upSlide.getCurrentPosition() == 0){
                   return false;
               }
               else{
                   return true;
               }
           }
           public Action
        }
   /* public class Out{
        private DcMotor outSlide;
        public class
        public Out(HardwareMap hardwareMap){
            outSlide =  hardwareMap.get(DcMotor.class, "out");
            outSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        }
        public boolean run(@NonNull TelemetryPacket packet){

        }
    }*//*
    public class MiniArm {
        private Servo arm;
        public MiniArm(HardwareMap hardwareMap){
            arm = hardwareMap.get(Servo.class, "arm");
        }
    }

    public class upClaw{
        private Servo verticalClaw;
        public upClaw(HardwareMap hardwaremap){
            verticalClaw = hardwaremap.get(Servo.class, "upClaw");
        }
    }

    /*public class outclaw{
        private Servo beetleLol;
        public outclaw(HardwareMap hardwareMap){
            beetleLol = hardwareMap.get(Servo.class, "outClawServo");
        }
    }
    public class outClawRotation{
        private Servo outClawRotate;
        public outClawRotation(HardwareMap hardwareMap){
            outClawRotate = hardwareMap.get(Servo.class, "outClawRotationServo");
        }
    }
*/

/*
    }

}



*/


