package org.firstinspires.ftc.teamcode.opmodes;


import androidx.annotation.NonNull;

// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.MecanumDrive;

@Config
@Autonomous(name = "TRUMAN'S_SKIBIDI_AUTO_MODE_16PTS", group = "Autonomous")
public class LeAutonomous extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    private Servo wrist;
    public void runOpMode() throws InterruptedException {
        waitForStart();
        topLeftMotor = hardwareMap.get(DcMotor.class,"leftFront");
        topRightMotor = hardwareMap.get(DcMotor.class,"rightFront");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"leftBack");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"rightBack");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor lift = hardwareMap.get(DcMotor.class,"lift");

        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        arm.setTargetPosition(0);
        arm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        arm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        CRServo claw = hardwareMap.get(CRServo.class, "intake");

        wrist = hardwareMap.get(Servo.class, "wrist");

        wrist.setDirection(Servo.Direction.FORWARD);
        wrist.scaleRange(0.1,0.8);

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
          lift.setTargetPosition(-4300);
          lift.setPower(0.5);
          sleep(5000);
          telemetry.addData("arm goed down","");
          telemetry.update();
        arm.setTargetPosition(-700);

        arm.setPower(0.6);
        sleep(2000);

          wrist.setPosition(0.1);

          sleep(500);

         wrist.setPosition(0.8);

          sleep(500);
          arm.setTargetPosition(0);
          sleep(500);
          arm.setPower(0);
          while (lift.getCurrentPosition() < -10) {
            lift.setPower(0.2);
            lift.setTargetPosition(lift.getCurrentPosition() + 40);
            //  sleep(10);
          }
          lift.setTargetPosition(-10);

    }
}
