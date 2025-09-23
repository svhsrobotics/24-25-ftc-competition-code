package org.firstinspires.ftc.teamcode.opmodes.intoTheDeep;

import static org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.SparkFunOTOSDrive.NewDrive;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.robotcore.hardware.HardwareMap;


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
