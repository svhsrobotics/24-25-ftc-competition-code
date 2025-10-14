package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Autonomous
public class TestAutoDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive drive = SparkFunOTOSDrive.NewDrive(hardwareMap, beginPose);
        Action trajAction = drive.actionBuilder(beginPose)
            .strafeTo(new Vector2d(100, 100)).build();

        waitForStart();

        Actions.runBlocking(trajAction);
    }
}
