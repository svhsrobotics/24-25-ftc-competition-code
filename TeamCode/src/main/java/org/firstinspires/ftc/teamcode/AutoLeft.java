package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
public class AutoLeft extends LinearOpMode {
    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftLiftMotor;
    //private DcMotor rightLiftMotor;
    private DcMotor intakeSlide;
    private Servo intakeElbow;
    private Servo intakeClaw;
    private Servo outtakeElbow;
    private Servo outtakeClaw;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private double liftPosition;
    private boolean driveSlow;


    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift");
        //rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift");
        intakeSlide = hardwareMap.get(DcMotor.class, "intake_slide");
        intakeElbow = hardwareMap.get(Servo.class, "intake_wrist");
        intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        outtakeElbow = hardwareMap.get(Servo.class, "deposit_wrist");
        outtakeClaw = hardwareMap.get(Servo.class, "outtake_grab");


        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        double elbowPosition = 0;
        boolean holdingPosition = false;
        driveSlow = false;


        waitForStart();
        intakeElbow.setPosition(.8);

        for (int i = 0; i < 3800; i++) {
            leftBackMotor.setPower(-1);
            rightBackMotor.setPower(-1);
            leftFrontMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
        }
//        for (int l = 0; l < 1500; l++) {
//            leftBackMotor.setPower(-1);
//            rightBackMotor.setPower(1);
//            leftFrontMotor.setPower(-1);
//            rightFrontMotor.setPower(1);
//        }



        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);

        leftLiftMotor.setTargetPosition(2000);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftLiftMotor.setPower(1);
        sleep(2000);
        for (int l = 0; l < 2500; l++) {
            leftBackMotor.setPower(-1);
            leftFrontMotor.setPower(-1);
        }
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        sleep(1000);
        outtakeElbow.setPosition(.5);
        sleep(3000);
        outtakeClaw.setPosition(1);
        sleep(2000);
        outtakeElbow.setPosition(0);
        sleep(1000);
        leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        leftLiftMotor.setPower(0);




    }
}
