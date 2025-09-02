package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
public class AutoAttemptIPromise extends LinearOpMode {


    AprilTagDetection detection;

    double myX = detection.robotPose.getPosition().x;
    double myY = detection.robotPose.getPosition().y;
    double myZ = detection.robotPose.getPosition().z;

    double myPitch = detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES);
    double myRoll = detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES);
    double myYaw = detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES);


    @Override
    public void runOpMode() throws InterruptedException {

    }
}
