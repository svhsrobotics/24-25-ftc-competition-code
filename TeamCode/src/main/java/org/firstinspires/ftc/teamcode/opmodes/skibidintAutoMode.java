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