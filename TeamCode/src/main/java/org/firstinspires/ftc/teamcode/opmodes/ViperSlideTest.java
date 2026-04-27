package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class ViperSlideTest extends OpMode {

    DcMotor left;
    DcMotor right;
    DcMotor slide;
    double x;
    double y;
    double extend;
    double extendSpeed;
    boolean dPadPressed;

    @Override
    public void init() {
        left = hardwareMap.get(DcMotor.class, "left_drive");
        right = hardwareMap.get(DcMotor.class, "right_drive");
        slide = hardwareMap.get(DcMotor.class, "slideMotor");

        left.setDirection(DcMotor.Direction.REVERSE);
        right.setDirection(DcMotor.Direction.FORWARD);
        slide.setDirection(DcMotor.Direction.FORWARD);

        x = 0;
        y = 0;
        extend = 0;
        extendSpeed = 15;
        dPadPressed = false;
    }

    @Override
    public void loop() {
        y = -gamepad1.left_stick_y;
        x = gamepad1.left_stick_x;

        left.setPower(y + x);
        right.setPower(y - x);

        if (gamepad1.left_bumper) {
            extend = extendSpeed/2;
        }
        if (gamepad1.right_bumper) {
            extend = -extendSpeed;
        }
        if ((gamepad1.left_bumper && gamepad1.right_bumper) ||
                (!gamepad1.left_bumper && !gamepad1.right_bumper)) {
            extend = 0;
        }

        slide.setPower(extend/100);

        if (gamepad1.dpad_right && !dPadPressed) {
            dPadPressed = true;
            extendSpeed += 1;
        }
        if (gamepad1.dpad_left && !dPadPressed) {
            dPadPressed = true;
            extendSpeed -= 1;
        }
        if (!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
            dPadPressed = false;
        }

        telemetry.addData("extendSpeed: ", extendSpeed);
        telemetry.update();
    }
}
