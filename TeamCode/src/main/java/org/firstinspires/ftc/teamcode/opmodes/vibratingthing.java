//
//i asked google for an apriltags test code and this is what it gave me
//
package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp
public class vibratingthing extends LinearOpMode {
    private Gamepad.RumbleEffect ja;

    @Override
    public void runOpMode() {
        waitForStart();

        while (opModeIsActive()) {
            ja = new Gamepad.RumbleEffect.Builder()
                            .addStep(1,0,500)
                                    .addStep(0,1,500)
                                            .build();
            gamepad1.runRumbleEffect(ja);

        }
    }
}