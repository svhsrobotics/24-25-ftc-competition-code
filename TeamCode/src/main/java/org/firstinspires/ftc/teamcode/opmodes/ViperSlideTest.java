package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@TeleOp
public class ViperSlideTest extends OpMode {

    DcMotor slide;
    double extend;
    double extendSpeed;
    boolean dPadPressed;

    @Override
    public void init() {
        slide = hardwareMap.get(DcMotor.class, "slideMotor");

        slide.setDirection(DcMotor.Direction.FORWARD);

        extend = 0;
        extendSpeed = 15;
        dPadPressed = false;
    }

    @Override
    public void loop() {
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
