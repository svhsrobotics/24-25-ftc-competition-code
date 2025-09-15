//
//i asked google for an apriltags test code and this is what it gave me
//
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@TeleOp(name = "Concept: AprilTag", group = "Concept")
public class bleh extends LinearOpMode {

    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;

    @Override
    public void runOpMode() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                .build();

        // Create the VisionPortal, and pass it the AprilTag processor.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1")) // Replace "Webcam 1" with your webcam's name
                .addProcessor(aprilTag)
                .build();

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            List<AprilTagDetection> currentDetections = aprilTag.getDetections();

            // Iterate through the detected AprilTags and display their information.
            for (AprilTagDetection detection : currentDetections) {
                if (detection.metadata != null) {
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Tag Name", detection.metadata.name);
                    telemetry.addData("Range", detection.ftcPose.range);
                    telemetry.addData("Bearing", detection.ftcPose.bearing);
                    telemetry.addData("Elevation", detection.ftcPose.elevation);
                } else {
                    telemetry.addData("Tag ID", detection.id);
                    telemetry.addData("Metadata", "Not available (tag size not provided)");
                }
            }
            telemetry.update();
        }

        // Shut down the VisionPortal.
        visionPortal.close();
    }
}