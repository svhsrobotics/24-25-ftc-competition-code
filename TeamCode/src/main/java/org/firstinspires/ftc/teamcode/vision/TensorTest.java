package org.firstinspires.ftc.teamcode.vision;


import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
@Config
@Disabled
public class TensorTest extends LinearOpMode {






    @Override
    public void runOpMode() throws InterruptedException {
        AprilTagCamera[] cameras = new AprilTagCamera[3];
        //cameras[0] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Left"), 8, Math.toRadians(70), Math.toRadians(-45));
        //cameras[1] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Center"), 7, Math.toRadians(90), Math.toRadians(0));
        //cameras[2] = new AprilTagCamera(hardwareMap.get(WebcamName.class, "Right"), 8, Math.toRadians(-70), Math.toRadians(45));




        TensorFlowDetection tensor = new TensorFlowDetection(cameras[1].webcamName);


        TensorFlowDetection.PropPosition tensorPos = null;


        android.util.Log.i("Location", tensorPos + "");

    }
}
