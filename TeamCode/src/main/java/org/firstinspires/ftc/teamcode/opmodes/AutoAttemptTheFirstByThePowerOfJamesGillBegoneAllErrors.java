package org.firstinspires.ftc.teamcode.opmodes;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.util.PIDController2;

@Config
@Autonomous
public class AutoAttemptTheFirstByThePowerOfJamesGillBegoneAllErrors extends LinearOpMode {
    public class Lift {
        private DcMotor upSlide;
        private PIDController2 PID = new PIDController2(1.2 * 0.02 / 1.28, 0.02 * 0.6, 0.075 * 0.02 * 1.28, 1);

        public Lift(HardwareMap hardwareMap) {

            upSlide = hardwareMap.get(DcMotor.class, "notheight");
            upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            upSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        public class highBasket implements Action {


            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                upSlide.setPower(PID.usePIDLoop(upSlide.getCurrentPosition(), 4000));
                if (upSlide.getCurrentPosition() == 4000) {
                    return false;
                } else {
                    return true;
                }


            }}

            public Action toHighBasket() {
                return new highBasket();
            }

        public class UpGoToZero implements Action {
            public boolean run(@NonNull TelemetryPacket packet) {
                upSlide.setPower(PID.usePIDLoop(upSlide.getCurrentPosition(), 0));

                if (upSlide.getCurrentPosition() == 0) {
                    return false;
                } else {
                    return true;
                }
            }

        }

        public Action upToZero() {
            return new UpGoToZero();
        }
    }
        public class Out {
            private DcMotor outSlide;


            public Out(HardwareMap hardwareMap) {
                outSlide = hardwareMap.get(DcMotor.class, "out");
                outSlide.setDirection(DcMotorSimple.Direction.REVERSE);
            }


        }

        public class MiniArm {
            private Servo arm;


            public MiniArm(HardwareMap hardwareMap) {
                arm = hardwareMap.get(Servo.class, "arm");
            }

            public class ResetArmPos implements Action{

                public boolean run(@NonNull TelemetryPacket packet){
                    arm.setPosition(0);
                    return false;
                }
            }
            public Action resetPos(){
                return new ResetArmPos();
            }

            public class RotateByPoint1 implements Action {
                public boolean run(@NonNull TelemetryPacket packet) {
                    arm.setPosition(arm.getPosition() + 0.1);
                    return false;
                }
            }
            public Action rotateByPoint1(){
                return new RotateByPoint1();
            }



            public class RotateByMinusPoint1 implements Action{


                public boolean run(@NonNull TelemetryPacket packet) {
                    arm.setPosition(arm.getPosition() - 0.1);
                    return false;
                }
            }
                public Action rotateByPointMinus1() {
                    return new RotateByPoint1();
                }



        }


    public class theUpClaw {
        private Servo verticalClaw;

        public theUpClaw(HardwareMap hardwaremap) {
            verticalClaw = hardwaremap.get(Servo.class, "upClaw");
        }
        public class OpenClaw implements Action{
            public boolean run(@NonNull TelemetryPacket packet){
                verticalClaw.setPosition(0.7);
                return false;
            }
        }



        public Action openClaw() {
            return new OpenClaw();
        }

        public class CloseClaw implements Action{
            public boolean run(@NonNull TelemetryPacket packet){
                verticalClaw.setPosition(1);
                return false;
            }
        }

        public Action closeClaw(){
            return new CloseClaw();
        }
    }

    public class outclaw {
        private Servo beetleLol;

        public outclaw(HardwareMap hardwareMap) {
            beetleLol = hardwareMap.get(Servo.class, "outClawServo");
        }
    }

    public class outClawRotation {
        private Servo outClawRotate;

        public outClawRotation(HardwareMap hardwareMap) {
            outClawRotate = hardwareMap.get(Servo.class, "outClawRotationServo");
        }
    }


    @Override
    public void runOpMode() {
        Pose2d initialPose = new Pose2d(10, 60, Math.toRadians(90));
        SparkFunOTOSDrive drive = SparkFunOTOSDrive.NewDrive(hardwareMap, initialPose );
        Lift lift = new Lift(hardwareMap);
         MiniArm arm = new MiniArm(hardwareMap);
         theUpClaw claw = new theUpClaw(hardwareMap);

        TrajectoryActionBuilder tab1 =  drive.actionBuilder(initialPose)
                .lineToYSplineHeading(33, Math.toRadians(0))
                .waitSeconds(2)
                .setTangent(Math.toRadians(90))
                .lineToY(48)
                .strafeTo(new Vector2d(44.5, 30))
                .lineToX(47.5)
                .waitSeconds(3);


        Actions.runBlocking(
                new SequentialAction(
                        tab1.build(),
                        lift.upToZero(),
                        lift.toHighBasket(),
                        arm.resetPos(),
                        claw.openClaw()









                )
        );




    }
}






