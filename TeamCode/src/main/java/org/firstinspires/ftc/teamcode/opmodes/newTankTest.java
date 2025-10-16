package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Toggle;

@Config
@TeleOp
public class newTankTest extends LinearOpMode {


    private DcMotor right;
    private DcMotor left;

    @Override
    public void runOpMode() {

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");

        while (opModeIsActive()) {
            if (gamepad1.left_stick_y != 0) {

        }
    }





}}