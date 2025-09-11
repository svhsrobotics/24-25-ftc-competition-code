
package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class RacecarTestForDog extends LinearOpMode {





    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    Toggle tog = new Toggle();

    @Override
    public void runOpMode() throws InterruptedException {

        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front_left_dw");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back_right_dw");



        waitForStart();

        while (opModeIsActive()) {
            double frontBack;
            double leftRight;
            double angle;
            rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            rightBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            if(gamepad1.left_trigger!=0){
           frontBack = gamepad1.left_trigger;
            }
            else if(gamepad1.right_trigger != 0){
                frontBack = -gamepad1.right_trigger;
            }







            }
        }
    }

