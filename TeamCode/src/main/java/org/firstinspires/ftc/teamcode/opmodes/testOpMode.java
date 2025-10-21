
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.AimAssist;
import org.firstinspires.ftc.teamcode.util.Toggle;

import kotlin.math.UMathKt;

@Config
@TeleOp
public class testOpMode extends LinearOpMode {





    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    Toggle tog = new Toggle();
    AimAssist gun = new AimAssist();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");



        waitForStart();

        while (opModeIsActive()) {
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            double y = Math.pow(-gamepad1.left_stick_y, 3); // Remember, Y stick is reversed!
            double x = Math.pow(gamepad1.left_stick_x,3);
            double rx = Math.pow(-gamepad1.right_stick_x, 3);
            double leftFrontPower = (y + x + rx);

            leftFrontMotor.setPower(leftFrontPower);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);

        }
    }
}

