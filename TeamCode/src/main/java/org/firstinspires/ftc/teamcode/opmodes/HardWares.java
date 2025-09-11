package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
@Config
public class HardWares extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor Arm;
    private DcMotor elbow;
    private DcMotor viper;
    private PIDController2 PIDA;
    private PIDController2 PIDE;
    private PIDController2 PIDV;
    private Toggle tog;
    private Toggle mode;
    private Debouncer debA;
    private Debouncer debB;
    private int viperTarPos = 0;



    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        Arm = hardwareMap.get(DcMotor.class, "arm");
        elbow = hardwareMap.get(DcMotor.class, "elbow");
        viper = hardwareMap.get(DcMotor.class, "height");
        double encoderPositionA = 0;
        double encoderPositionE = 0;
        waitForStart();
        double power = 0;
        double iError = 0;
        double eError = 0;
        double ePower = 0;
        double eIError = 0;
        double ePrevError = 0;
        double eDiff = 0;
        double eKp = 0.005;
        double eKi = 0;
        double eKd = 0;
        double Kp = 0.005;
        double Ki = 0;
        double Kd = 0;
        Servo wrist;
        double servoPos = 0;


        double diff;
        double prevError;
        double armReference = 0;
        double elbowReference = 0;
        double viperReference = 0;
        int clawServo = 0;


        Servo SomethingServo;
        SomethingServo = hardwareMap.get(Servo.class, "whiteCable");
        wrist = hardwareMap.get(Servo.class, "blackCable");


        tog = new Toggle();
        mode = new Toggle();
        //remember what we did with classes


        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        viper.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        viper.setDirection(DcMotorSimple.Direction.REVERSE);



        PIDA = new PIDController2(Ki, Kp, Kd, 1);
        PIDE = new PIDController2( eKi, eKp, eKd, 1);
        PIDV = new PIDController2( 0, 0.05, 0,1);

        while (opModeIsActive()) {


            //arm stuff


            if (gamepad2.dpad_up || gamepad1.dpad_up) {
                armReference = armReference + 4;
            } else if (gamepad2.dpad_down || gamepad1.dpad_down) {
                armReference = armReference - 4;
            }

            //arm limits so that it doesnt go backwards

            if (armReference > 2500) {
                armReference = 2500;
            }
            if (!(armReference <= 0)) {
                Arm.setPower(PIDA.usePIDLoop(Arm.getCurrentPosition(), armReference));
            } else {
                Arm.setPower(0);
            }
            //elbow stuff

            if (gamepad2.dpad_left || gamepad1.dpad_left) {
                elbowReference = elbowReference + 7;
            } else if (gamepad2.dpad_right || gamepad1.dpad_right) {
                elbowReference = elbowReference - 7;
            }

           /* if (elbowReference > 0) {
                elbowReference = 0;
            }*/
            elbow.setPower(PIDE.usePIDLoop(elbow.getCurrentPosition(), elbowReference));

            //claw stuff

            tog.update(gamepad2.right_bumper);
            tog.update(gamepad1.right_bumper);
            if (!tog.state) {
                SomethingServo.setPosition(0);
                clawServo = 0;
            } else {
                SomethingServo.setPosition(1);
                clawServo = 1;

            }


            //wrist stuff


            //wrist.setPosition(servoPos);


            if (gamepad2.a || gamepad1.a) {

                servoPos = servoPos - 0.01;
            } else if (gamepad2.b || gamepad1.b) {

                servoPos = servoPos + .01;

            }

            //
            wrist.setPosition(servoPos);
            //

            if (servoPos > 1) {
                servoPos = 1;
            } else if (servoPos < 0) {
                servoPos = 0;
            }

            //viperslide


            //START HERE
            if (gamepad2.x || gamepad1.x) {

                viperTarPos = viperTarPos + 5;
            } else if (gamepad2.y || gamepad1.y) {
                viperTarPos = viperTarPos - 5;
            }
            if (viperTarPos > 1066 && Arm.getCurrentPosition() <= 1600 && elbow.getCurrentPosition() <30) {
                viperTarPos = 30;
            }
            else  if (viperTarPos > 1066 && Arm.getCurrentPosition() <= 1600) {
                viperTarPos = 1065;
            }


            boolean pos;


            pos = Arm.getCurrentPosition() > 830 && Arm.getCurrentPosition() < 1000;

            if(pos  && elbow.getCurrentPosition() > -300){
                viperTarPos = 0;
            elbowReference = -300;
            armReference = 830;}




            viper.setTargetPosition(viperTarPos);
            //viper.setTargetPosition(100);
            viper.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            viper.setPower(1);




/*
                if(gamepad2.x){
                    viperReference = viperReference +5;
                }
                else if(gamepad2.y){
                    viperReference = viperReference -5;
                }

                 viper.setPower(PIDV.usePIDLoop(viper.getCurrentPosition(), viperReference));
*/

            //mode.usePIDLoop(gamepad2.left_stick_button);

            //if (mode.state) {


                telemetry.addData("Arm Pos", Arm.getCurrentPosition());
                telemetry.addData("a reference", armReference);
                telemetry.addData("elbow pos", elbow.getCurrentPosition());
                telemetry.addData("e reference", elbowReference);
                telemetry.addData("ystick", gamepad2.left_stick_y);
                telemetry.addData("viperticks", viper.getCurrentPosition());
                telemetry.addData("viperTarPos", viperTarPos);



                telemetry.addData("servoPos", servoPos);
                telemetry.update();

                //drive stuff


                //}
            //}


            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y1 = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x1 = gamepad1.left_stick_x;
            double rx1 = gamepad1.right_stick_x;
            double y2 = -gamepad2.left_stick_y; // Remember, Y stick is reversed!
            double x2 = gamepad2.left_stick_x;
            double rx2 = gamepad2.right_stick_x;
            telemetry.addData("y1", y1);
            telemetry.addData("x1", x1);
            telemetry.addData("rx1", rx1);
            telemetry.addData("y2", y2);
            telemetry.addData("x2", x2);
            telemetry.addData("rx2", rx2);

            if(gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0){


            leftFrontMotor.setPower(y1 + x1 + rx1);
            leftBackMotor.setPower(y1 - x1 + rx1);
            rightFrontMotor.setPower(y1 - x1 - rx1);
            rightBackMotor.setPower(y1 + x1 - rx1);
        }
            else  {
                leftFrontMotor.setPower(y2 + x2 + rx2);
                leftBackMotor.setPower(y2 - x2 + rx2);
                rightFrontMotor.setPower(y2 - x2 - rx2);
                rightBackMotor.setPower(y2 + x2 - rx2);
            }
            //macro attempt
            /*if (gamepad1.left_bumper || gamepad2.left_bumper) {
                Arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                Arm.setTargetPosition(2000);
                Arm.setPower(0.2);
                elbow.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                elbow.setTargetPosition(-1148);
                elbow.setPower(0.2);
               viperTarPos = 1800;

            }*/
    }
}}