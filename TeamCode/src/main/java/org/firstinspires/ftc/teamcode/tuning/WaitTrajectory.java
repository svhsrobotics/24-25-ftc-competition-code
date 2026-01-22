package org.firstinspires.ftc.teamcode.tuning;

//import static org.firstinspires.ftc.teamcode.tuning.otos.SparkFunOTOSDrive.NewDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.tuning.otos.SparkFunOTOSDrive;


public class WaitTrajectory {

    Pose2d initialPose;
    SparkFunOTOSDrive drive;

    public WaitTrajectory(HardwareMap hardwareMap, Pose2d initialPose) {
        this.initialPose = initialPose;
        drive = NewDrive(hardwareMap, initialPose);
    }

    public Action waitSeconds(double seconds) {
        return drive.actionBuilder(initialPose)
                .waitSeconds(seconds).build();
    }

}
