package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.message.redux.InitOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public abstract class AutoAttemptTwoByThePowerOfTheBookAndInTheNameOfJamesGill extends OpMode {
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;


        WebcamName webcamName = hardwareMap.get(WebcamName.class, "webcam1");
    @Override
    public void init(){
     visionPortal = new VisionPortal.Builder()
                .addProcessor(aprilTagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640, 480))
                .setStreamFormat(VisionPortal.StreamFormat.YUY2)
                .enableLiveView(true)
                .setAutoStopLiveView(true)


                .build();
    }
    public void init_loop(){
        List<AprilTagDetection> currentDetections=aprilTagProcessor.getDetections();
        StringBuilder idsFound = new StringBuilder();
        for(AprilTagDetection detection : currentDetections){
            idsFound.append(detection.id);
            idsFound.append(' ');
        }
        telemetry.addData("aprilTags", idsFound);

    }
    @Override
    public void start(){
        visionPortal.stopStreaming();

    }


}
