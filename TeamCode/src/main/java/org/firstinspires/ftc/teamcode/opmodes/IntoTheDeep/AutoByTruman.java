package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;

import android.os.Build;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.Trajectory;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.TrajectoryActionFactory;
import com.acmerobotics.roadrunner.TrajectoryBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

import java.net.Authenticator;

@Config
@Autonomous
public class AutoByTruman extends LinearOpMode {
    private class launcher {

    }
    //GOAL: identify the color of ball our robot needs and find it then shoot it in the basket
    //and do this 3 times to get the combination on the apriltag btw it has to read the apriltags


    //if it doesnt work, try putting build before the runblocking thing. for the futre

    //i'm still working on ts. dont let nick or that brendan kid touch the code


    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(0, 0, Math.toRadians(0));

        SparkFunOTOSDrive drive = SparkFunOTOSDrive.NewDrive(hardwareMap, initialPose);

        TrajectoryActionBuilder ya =  drive.actionBuilder(initialPose)
                .splineTo(new Vector2d(2,2),1);


        Actions.runBlocking(
                new SequentialAction(
                        ya.build()

                )
        );
    }
}
