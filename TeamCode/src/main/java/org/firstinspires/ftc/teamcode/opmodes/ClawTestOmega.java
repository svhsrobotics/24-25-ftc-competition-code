package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

//@TeleOp
public class ClawTestOmega extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Double power = 0.3;

        //Servo claw = hardwareMap.get(Servo.class,"intake");
        CRServo claw = hardwareMap.get(CRServo.class, "intake");
        DcMotor arm = hardwareMap.get(DcMotor.class,"arm");
        boolean buttonbeingpressed = false;

        waitForStart();

        //claw.setDirection(Servo.Direction.FORWARD);
        claw.setDirection(DcMotorSimple.Direction.FORWARD);
        while (opModeIsActive()) {
            // telemetry.addData("claw position",arm.getCurrentPosition());
            // telemetry.update();
            if (gamepad1.a && buttonbeingpressed) {
                buttonbeingpressed = false;
            }
            else if (gamepad1.a && !buttonbeingpressed) {
                buttonbeingpressed = true;
            }
            if (buttonbeingpressed) {
                claw.setPower(1);
            }
            else{
              claw.setPower(0);
            }

        }
    }
}
