
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class TestForDog extends LinearOpMode {





    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private Servo SomethingServo;
    private Servo wrist;
    Toggle tog = new Toggle();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_right_dw");
        SomethingServo = hardwareMap.get(Servo.class, "CHANGETHISORURSTUPID");
        wrist = hardwareMap.get(Servo.class, "wrist");
        double servoPos = 0.5;


        waitForStart();
        while (opModeIsActive()) {
            leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x;
            double rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

            tog.update(gamepad1.right_bumper);

            if(tog.state){
                SomethingServo.setPosition(0);
            }

            else {
                SomethingServo.setPosition(1);

            }
            if (gamepad2.left_bumper){
                wrist.setPosition(servoPos);
                servoPos = servoPos - .01;

            }
            if (gamepad2.right_bumper){
                wrist.setPosition(servoPos);
                servoPos = servoPos + .01;

            }
            telemetry.addData("right button", gamepad2.right_bumper);
            telemetry.addData("left bumper", gamepad2.left_bumper);
            telemetry.update();

            }
        }
    }

