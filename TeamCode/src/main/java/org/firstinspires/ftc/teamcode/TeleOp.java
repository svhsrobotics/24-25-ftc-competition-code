package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TeleOp extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor leftLiftMotor;
    private DcMotor rightLiftMotor;
    private DcMotor intakeSlide;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private double liftPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        leftLiftMotor = hardwareMap.get(DcMotor.class, "left_lift");
        rightLiftMotor = hardwareMap.get(DcMotor.class, "right_lift");
        intakeSlide = hardwareMap.get(DcMotor.class, "intake_slide");

        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();
        while (opModeIsActive()) {
            y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            x = gamepad1.right_stick_x;
            rx = gamepad1.left_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

            if (gamepad1.right_bumper) {
                leftLiftMotor.setPower(0.3);
                rightLiftMotor.setPower(0.3);
            } else if (gamepad1.left_bumper) {
                leftLiftMotor.setPower(0.1);
                rightLiftMotor.setPower(0.1);
            } else {
                leftLiftMotor.setPower(0);
                rightLiftMotor.setPower(0);
            }

            if (gamepad1.right_trigger > 0.1) {
                intakeSlide.setPower(0.5);
            } else if (gamepad1.left_trigger >0.1) {
                intakeSlide.setPower(-0.5);
            } else {
                intakeSlide.setPower(0);
            }

            if (gamepad1.a) {
                while (leftLiftMotor.getCurrentPosition() < 400 ||leftLiftMotor.getCurrentPosition() > 500) {
                    leftLiftMotor.setTargetPosition(500);
                    leftLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    leftLiftMotor.setPower(0.2);
                    rightLiftMotor.setTargetPosition(500);
                    rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    rightLiftMotor.setPower(0.2);
                }
            }

            liftPosition = leftLiftMotor.getCurrentPosition();


            telemetry.addData("LIFT POSITION", liftPosition);
            telemetry.addData("VIPER SLIDE POSITION", intakeSlide.getCurrentPosition());
            telemetry.update();

        }
    }

}
