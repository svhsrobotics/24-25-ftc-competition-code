package org.firstinspires.ftc.teamcode.opmodes;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.PIDController2;
@Config
@TeleOp
public class THETWO extends LinearOpMode {


        private DcMotor leftFrontMotor;
        private DcMotor rightFrontMotor;
        private DcMotor leftBackMotor;
        private DcMotor rightBackMotor;
        private DcMotor Arm;
        private DcMotor elbow;
        private DcMotor viper;
        private PIDController2 PIDA;

        @Override
        public void runOpMode() throws InterruptedException {

            leftFrontMotor = hardwareMap.get(DcMotor.class, "front_left");
            rightFrontMotor = hardwareMap.get(DcMotor.class, "front_right");
            leftBackMotor = hardwareMap.get(DcMotor.class, "back_left");
            rightBackMotor = hardwareMap.get(DcMotor.class, "back_right");
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
            double eKp = 0;
            double eKi = 0;
            double eKd = 0;
            double Kp = 0.005;
            double Ki = 0;
            double Kd = 0;

            double diff;
            double prevError;
            double armReference = 300;

            PIDA = new PIDController2(armReference, Kp, Ki, Kd);
            //remember what we did with classes

            double eReference = 0;
            double viperPower = 0;


            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            while (opModeIsActive()) {
                if (gamepad1.b) {



                    eError = eReference - encoderPositionE;


                    //reference is where we want to be

                    eReference = 0;


                    // obtain the encoder position

                    encoderPositionE = elbow.getCurrentPosition();


                    eIError = eIError + eError;
                    //cumulative error

                    eDiff = eError - ePrevError;
                    //calculate the D


                    eError = eReference - encoderPositionE;
                    // calculate the error


                    ePower = eKp * eError + eKi * eIError + eKd * eDiff;


                    //reference is where we want to be





                    Arm.setPower(PIDA.update(Arm.getCurrentPosition()));
                    elbow.setPower(ePower);

                    telemetry.addData("arm", encoderPositionA);

                    telemetry.addData("reference", armReference);
                    telemetry.addData("power", power);
                    telemetry.addData("elbow", encoderPositionE);
                    telemetry.addData("Error E", eError);
                    telemetry.addData("E Power", ePower);
                    telemetry.update();


                }
                else Arm.setPower(0);

                //driving

                leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
                double x = gamepad1.left_stick_x;
                double rx = gamepad1.right_stick_x;

                leftFrontMotor.setPower(y + x + rx);
                leftBackMotor.setPower(y - x + rx);
                rightFrontMotor.setPower(y - x - rx);
                rightBackMotor.setPower(y + x - rx);


            }
        }
    }




//3 hours for 1 bracket... always remember to click code->reformat code











