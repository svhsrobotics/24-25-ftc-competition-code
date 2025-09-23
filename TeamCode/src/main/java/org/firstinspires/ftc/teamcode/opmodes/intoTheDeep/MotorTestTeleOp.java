package org.firstinspires.ftc.teamcode.opmodes.intoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class MotorTestTeleOp extends LinearOpMode {

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
        //leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift");
        //rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift");
        //intakeSlide = hardwareMap.get(DcMotor.class, "intake_slide");
        //intakeElbow = hardwareMap.get(Servo.class, "intake_wrist");
        //intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        //outtakeElbow = hardwareMap.get(Servo.class, "deposit_wrist");
        //outtakeClaw = hardwareMap.get(Servo.class, "outtake_grab");


        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
       // leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //intakeSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //leftLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //intakeSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //double elbowPosition = 0;
        //boolean holdingPosition = false;
        //driveSlow = false;


        waitForStart();
        while (opModeIsActive()) {
            if (gamepad1.b) {
                rightBackMotor.setPower(1);

            }
            else if (gamepad1.a) {
                leftBackMotor.setPower(1);
            }
else if (gamepad1.x) {
    leftFrontMotor.setPower(1);
}
else if (gamepad1.y) {
    rightFrontMotor.setPower(1);
}
else {
                rightBackMotor.setPower(0);
                leftBackMotor.setPower(0);
                leftFrontMotor.setPower(0);
                rightFrontMotor.setPower(0);


            }
        }
    }

}
