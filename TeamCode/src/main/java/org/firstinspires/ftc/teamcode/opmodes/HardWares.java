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
    private Debouncer debA;
    private Debouncer debB;

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
         double servoPos = 0.5;



        double diff;
        double prevError;
        double armReference = 0;
        double elbowReference = 0;
        double viperReference = 0;


        Servo SomethingServo;
        SomethingServo = hardwareMap.get(Servo.class, "whiteCable");
        wrist = hardwareMap.get(Servo.class, "blackCable");


        tog = new Toggle();
        //remember what we did with classes


        Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        PIDA = new PIDController2(armReference, Ki, Kp, Kd);
        PIDE = new PIDController2(elbowReference, eKi, eKp, eKd);

        while (opModeIsActive()) {


            //arm stuff


            if (gamepad2.dpad_up) {
                armReference = armReference + 10;
            } else if (gamepad2.dpad_down) {
                armReference = armReference - 10;
            }
            Arm.setPower(PIDA.update(Arm.getCurrentPosition(), armReference));

            //elbow stuff

            if (gamepad2.dpad_left) {
                elbowReference = elbowReference + 10;
            } else if (gamepad2.dpad_right) {
                elbowReference = elbowReference - 10;
            }
            elbow.setPower(PIDE.update(elbow.getCurrentPosition(), elbowReference));

            //claw stuff

            tog.update(gamepad2.right_bumper);

            if (tog.state) {
                SomethingServo.setPosition(0);
            } else {
                SomethingServo.setPosition(1);

            }

            //wrist stuff



            //wrist.setPosition(servoPos);
            /*TODO:debA.update(gamepad2.a);
            TODO:debB.update(gamepad2.b);*/

            if (gamepad2.b) {

                servoPos = servoPos - 0.1;

            }

            else if (gamepad2.a) {

                servoPos = servoPos + .1;

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

            if (gamepad2.left_stick_y > 0.03) {
                viper.setPower(0.01);
            } else if (gamepad2.left_stick_y < -0.03) {
                viper.setPower(-0.01);
            } else {
                viper.setPower(0);
            }
                /*

                if(gamepad2.dpad_up){
                    viperReference = viperReference +5;
                }
                else if(gamepad2.dpad_down){
                    viperReference = viperReference -5;
                }
                 viper.setPower(PIDV.update(viper.getCurrentPosition(), viperReference));

                 */


            telemetry.addData("Arm Pos", Arm.getCurrentPosition());
            telemetry.addData("a reference", armReference);
            telemetry.addData("elbow pos", elbow.getCurrentPosition());
            telemetry.addData("e reference", elbowReference);

            telemetry.addData("trigger1", gamepad2.left_trigger);
            telemetry.addData("trigger2", gamepad2.right_trigger);
            telemetry.addData("servoPos", servoPos);
            telemetry.update();

            //drive stuff


            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

            //}
        }
    }
}