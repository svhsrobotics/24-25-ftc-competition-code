package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class Amphetrite extends LinearOpMode {
    private DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
    private DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
    private DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
    private DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    private DcMotor horizontal = hardwareMap.get(DcMotor.class, "out");
    private DcMotor vertical = hardwareMap.get(DcMotor.class,  "height");
    private DcMotor vertical2 = hardwareMap.get(DcMotor.class, "height2");
    
    private Servo clawServo = hardwareMap.get(Servo.class, "whiteCable");
    private Servo wristServo = hardwareMap.get(Servo.class, "blackCable");
    private Servo wristTiltServo = hardwareMap.get(Servo.class, "tilt");

    private int vertTarPos = 0;
    private int horTarPos = 0;

    private PIDController2 pidV = new PIDController2(0, 0, 0, 0);
    private PIDController2 pidH = new PIDController2(0, 0, 0, 0);
    private Toggle clawToggle = new Toggle();
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if (gamepad1.left_trigger != 0) {//omg this is so much easier with anything but the dpad
            vertTarPos = vertTarPos + (int) gamepad1.left_trigger;
        } else if (gamepad1.right_trigger != 0) {
            horTarPos = horTarPos + (int) gamepad1.right_trigger;
        }
        horizontal.setPower(pidH.update(horizontal.getCurrentPosition(), horTarPos));
        vertical.setPower(pidV.update(vertical.getCurrentPosition(), vertTarPos));
        vertical2.setPower(pidV.update(vertical.getCurrentPosition(), vertTarPos));
        //claw
        clawToggle.update(gamepad1.right_bumper);
        if (!clawToggle.state){
            clawServo.setPosition(0);
        } else{
            clawServo.setPosition(1);
        }
        //wrist twist
        if(gamepad1.x){
            wristTiltServo.setPosition(wristTiltServo.getPosition() +0.01);
        } else if (gamepad1.b){
            wristTiltServo.setPosition(wristTiltServo.getPosition() -0.01);
        }
        //pitch
        if(gamepad1.a){
           wristTiltServo.setPosition(wristTiltServo.getPosition() -0.01);
        } else if (gamepad1.y) {
            wristTiltServo.setPosition(wristTiltServo.getPosition() -0.01);

        }

        //drive
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        double y1 = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
        double x1 = gamepad1.left_stick_x;
        double rx1 = gamepad1.right_stick_x;
        leftFrontMotor.setPower(y1 + x1 + rx1);
        leftBackMotor.setPower(y1 - x1 + rx1);
        rightFrontMotor.setPower(y1 - x1 - rx1);
        rightBackMotor.setPower(y1 + x1 - rx1);
        //telemetry
        telemetry.addLine("wrist positions");
        telemetry.addData("pitch", wristTiltServo);
        telemetry.addData("wristPos", wristServo);
        telemetry.addData("clawPos", clawServo);
        telemetry.addLine("Arm Positions");
        telemetry.addData("horizontalPos", horizontal.getCurrentPosition());
        telemetry.addData("verticalPos", vertical.getCurrentPosition());
        telemetry.addData("2ndVerticalPos", vertical2.getCurrentPosition());
        telemetry.addData("horizontal reference", horTarPos);
        telemetry.addData("vertical reference", vertTarPos);
        telemetry.addLine("extra");
        telemetry.addData("clawTogState", clawToggle.state);
        telemetry.update();


    }
}
