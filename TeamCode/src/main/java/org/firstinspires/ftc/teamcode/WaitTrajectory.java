package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SparkFunOTOSDrive.NewDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class WaitTrajectory {

    Pose2d initialPose;
    SparkFunOTOSDrive drive;

    public WaitTrajectory(HardwareMap hardwareMap) {
        initialPose = new Pose2d(-36, -60, Math.toRadians(0));
        drive = NewDrive(hardwareMap, initialPose);
    }

    public Action waitSeconds(double seconds) {
        return drive.actionBuilder(initialPose)
                .waitSeconds(seconds).build();
    }

}
