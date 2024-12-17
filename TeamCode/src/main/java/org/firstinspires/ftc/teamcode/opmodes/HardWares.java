package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private Toggle tog;

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        while (opModeIsActive()) {

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

            double diff;
            double prevError;
            double armReference = 300;
            double elbowReference = 300;


            tog = new Toggle();
            //remember what we did with classes

            double eReference = 0;
            double viperPower = 0;


            Arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            Arm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            elbow.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            elbow.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (gamepad1.a) {
                PIDA = new PIDController2(armReference, Ki, Kp, Kd);
                PIDE = new PIDController2(elbowReference, eKi, eKp, eKd);

                if (gamepad1.dpad_up) {
                    armReference = armReference + 3;
                } else if (gamepad1.dpad_down) {
                    armReference = armReference - 3;
                }

            }
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