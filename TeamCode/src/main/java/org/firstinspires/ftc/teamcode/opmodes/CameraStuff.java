package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class CameraStuff {

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    List<AprilTagDetection> witnessedTags = new ArrayList<>();
    Telemetry tagInfo;

    public void init(HardwareMap hwMap, Telemetry tagInfo) {
        this.tagInfo = tagInfo;
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        witnessedTags = tagProcessor.getDetections();
    }

    public List<AprilTagDetection> getWitnessedTags() {
        return witnessedTags;
    }

    public AprilTagPoseFtc returnFTCPose(AprilTagDetection detectedID) {
        if (detectedID == null) {return null;}
        return detectedID.ftcPose;
    }


    public void returnDetectionTelemetry(AprilTagDetection detectedID) {
        if (detectedID == null) {return;}
            if (detectedID.metadata == null) {
               // detectedID.ftcPose;
            }
        }

    public AprilTagDetection getTagID(int id) {
        for (AprilTagDetection detection : witnessedTags) {
            if (id == detection.id) {
                return detection;
            }
        }
        return null;
    }

    public void staph() {
        if (visionPortal != null) {
            visionPortal.close();
        }
    }
}
