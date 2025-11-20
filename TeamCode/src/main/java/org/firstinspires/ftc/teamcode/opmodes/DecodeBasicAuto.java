package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous
@Config
public class DecodeBasicAuto extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor smallLaunchMotor;

    private DcMotor bigLaunchMotor;
    private DcMotor intakeMotor;
    private Servo leftTrigger;
    private Servo rightTrigger;
    int g;
    int z;

    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        smallLaunchMotor = hardwareMap.get(DcMotor.class, "smallLaunchMotor");
        bigLaunchMotor = hardwareMap.get(DcMotor.class, "bigLaunchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bigLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        z =0;
        g=0;



        //setting launch motors to run with encoders

        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bigLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        smallLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//         dash = FtcDashboard.getInstance();
//        runningActions = new ArrayList<>();
//        initialPose = new Pose2d(1,0,0);
//        drive = NewDrive(hardwareMap, initialPose);
        leftTrigger.setPosition(.47);
        rightTrigger.setPosition(.52);
        waitForStart();
        for (int i = 0; i < 20000; i++) {
            leftFrontMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
            leftBackMotor.setPower(-1);
            rightBackMotor.setPower(-1);
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);
        smallLaunchMotor.setPower(.55);
        //big launch motor is 33% larger radius, so it must be reduced
        bigLaunchMotor.setPower(0.75 * .55);
        intakeMotor.setPower(0);

       sleep(2000); 
        leftTrigger.setPosition(.52);
        rightTrigger.setPosition(.455);
        for (int i = 0; i < 20000; i++) {
            leftFrontMotor.setPower(-1);
            rightFrontMotor.setPower(-1);
            leftBackMotor.setPower(1);
            rightBackMotor.setPower(1);
        }
        leftFrontMotor.setPower(0);
        rightFrontMotor.setPower(0);
        leftBackMotor.setPower(0);
        rightBackMotor.setPower(0);


    }
}

