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
    //drive motors
    private DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFrontMotor");
    private DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFrontMotor");
    private DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBackMotor");
    private DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBackMotor");
    //vertical arm
    private DcMotor intakeArm = hardwareMap.get(DcMotor.class, "out");
    private DcMotor vertical = hardwareMap.get(DcMotor.class,  "height");
    private DcMotor vertical2 = hardwareMap.get(DcMotor.class, "height2");
    //make a dcmotor vert claw
    private Servo clawServo = hardwareMap.get(Servo.class, "whiteCable");
    private Servo intakeWristServo = hardwareMap.get(Servo.class, "blackCable");
    private Servo intakeWristTiltServo = hardwareMap.get(Servo.class, "tilt");
    private Servo intakeServo = hardwareMap.get(Servo.class, "intakeServo");
    private Servo VertClawServo = hardwareMap.get(Servo.class, "VertClawServo");
    private Servo servoNumberABillion = hardwareMap.get(Servo.class, "vertFlipServo");
    //this servo just flips the vert claw

    private int vertTarPos = 0;
    private int horTarPos = 0;
    private int outtakeTuning = 20;

    private PIDController2 pidV = new PIDController2(0, 0, 0, 0);
    private PIDController2 pidH = new PIDController2(0, 0, 0, 0);
    private Toggle clawToggle = new Toggle();
    private boolean clawTogVar = false;
    //false means closed
    @Override
    public void runOpMode() throws InterruptedException {

        waitForStart();

        if (gamepad1.left_trigger != 0 && gamepad1.y)  {//omg this is so much easier with anything but the dpad
            vertTarPos = vertTarPos + (int) gamepad1.left_trigger;
        }
        else if(gamepad1.left_trigger != 0 && gamepad1.a){
            vertTarPos = vertTarPos - (int) gamepad1.left_trigger;
        }

       if (gamepad1.right_trigger != 0 && gamepad1.y) {
            horTarPos = horTarPos + (int) gamepad1.right_trigger;
        }
       else if(gamepad1.right_trigger != 0 && gamepad1.a){
           horTarPos = horTarPos - (int) gamepad1.right_trigger;
       }

        intakeArm.setPower(pidH.update(intakeArm.getCurrentPosition(), horTarPos));
        vertical.setPower(pidV.update(vertical.getCurrentPosition(), vertTarPos));
        vertical2.setPower(pidV.update(vertical.getCurrentPosition(), vertTarPos));
        //claw
        clawToggle.update(gamepad1.right_bumper);
        clawTogVar = clawToggle.state;
        if (!clawTogVar){
            clawServo.setPosition(0);
        } else{
            clawServo.setPosition(1);
        }
        //wrist twist
        if(gamepad1.x){
            intakeWristTiltServo.setPosition(intakeWristTiltServo.getPosition() +0.01);
        } else if (gamepad1.b){
            intakeWristTiltServo.setPosition(intakeWristTiltServo.getPosition() -0.01);
        }
        //pitch
        if(gamepad1.b){
            intakeWristTiltServo.setPosition(intakeWristTiltServo.getPosition() -0.01);
        } else if (gamepad1.y) {
            intakeWristTiltServo.setPosition(intakeWristTiltServo.getPosition() -0.01);

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
        telemetry.addData("pitch", intakeWristTiltServo);
        telemetry.addData("wristPos", intakeWristServo);
        telemetry.addData("clawPos", clawServo);
        telemetry.addLine("Arm Positions");
        telemetry.addData("intakeArm", intakeArm.getCurrentPosition());
        telemetry.addData("verticalPos", vertical.getCurrentPosition());
        telemetry.addData("2ndVerticalPos", vertical2.getCurrentPosition());
        telemetry.addData("horizontal reference", horTarPos);
        telemetry.addData("vertical reference", vertTarPos);
        telemetry.addLine("extra");
        telemetry.addData("clawTogState", clawToggle.state);
        telemetry.update();


        if (gamepad1.left_bumper) {
            horTarPos = 0;
            vertTarPos = 0;
            VertClawServo.setPosition(0);
            servoNumberABillion.setPosition(0);
            clawTogVar = false;
            intakeServo.setPosition(1);
            sleep(outtakeTuning);
            VertClawServo.setPosition(1);
            clawTogVar = true;
            //if this is broken, it's probably because the servo above needs to be flipped
            sleep(outtakeTuning);
            vertTarPos = 1200;
            servoNumberABillion.setPosition(1);


        }



    }
}
