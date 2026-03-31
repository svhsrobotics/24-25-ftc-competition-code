package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "Tank Drive", group = "TeleOp")
public class LucasRobotCode extends LinearOpMode {


    private DcMotor Motor;

    @Override
    public void runOpMode() {

        Motor  = hardwareMap.get(DcMotor.class, "Motor");

        Motor.setDirection(DcMotor.Direction.FORWARD);

        Motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized — ready to start!");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            double Power = gamepad1.right_stick_y;

            Motor.setPower(Power);

            telemetry.addData("Power:",  "%.2f", Power);

            telemetry.update();
        }
    }
}
