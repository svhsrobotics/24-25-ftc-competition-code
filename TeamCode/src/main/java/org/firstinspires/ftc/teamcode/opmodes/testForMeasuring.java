package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class testForMeasuring extends LinearOpMode {
    private DcMotor right;
    private DcMotor left;
    private DcMotor launch1;
    private DcMotor launch2;
    private DcMotor Spin2Win;
    private Servo gate;
    public void runOpMode(){
        Spin2Win = hardwareMap.get(DcMotor.class, "intake");
        gate = hardwareMap.get(Servo.class, "gateServo");
        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch1 = hardwareMap.get(DcMotor.class, "launch1");
        launch2 = hardwareMap.get(DcMotor.class, "launch2");
        waitForStart();
        while(opModeIsActive()){

            launch1.setPower(gamepad1.right_stick_y);
            launch2.setPower(launch1.getPower());

            Spin2Win.setPower(gamepad1.right_stick_y);


        }



    }

}
