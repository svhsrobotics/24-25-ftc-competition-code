package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;


@Config
@Autonomous(name = "TRUMAN'S_SKIBIDI_AUTO_MODE_3PTS", group = "Autonomous")
public class skibidintAutoMode extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    private Servo wrist;

    public void runOpMode() throws InterruptedException {
        waitForStart();
        topLeftMotor = hardwareMap.get(DcMotor.class, "leftFront");
        topRightMotor = hardwareMap.get(DcMotor.class, "rightFront");
        bottomLeftMotor = hardwareMap.get(DcMotor.class, "leftBack");
        bottomRightMotor = hardwareMap.get(DcMotor.class, "rightBack");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lift = hardwareMap.get(DcMotor.class, "lift");

        DcMotor arm = hardwareMap.get(DcMotor.class, "arm");
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CRServo claw = hardwareMap.get(CRServo.class, "intake");

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.scaleRange(0.1, 0.8);

        lift.setTargetPosition(0);

        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        topLeftMotor.setPower(0.4);
        bottomLeftMotor.setPower(0.4);
        topRightMotor.setPower(0.4);
        bottomRightMotor.setPower(0.4);
        sleep(500);

        topLeftMotor.setPower(0);
        bottomLeftMotor.setPower(0);
        topRightMotor.setPower(0);
        bottomRightMotor.setPower(0);

    }

}