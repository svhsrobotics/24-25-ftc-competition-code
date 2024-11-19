package org.firstinspires.ftc.teamcode.vision;

import com.acmerobotics.roadrunner.geometry.Pose2d;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

public class AprilTagCamera {
    public WebcamName webcamName;
    private final double forwardOffset;
    private final double sideOffset;

    public AprilTagCamera(WebcamName webcamName, double forwardOffset, double sideOffset) {
        this.webcamName = webcamName;
        this.forwardOffset = forwardOffset;
        this.sideOffset = sideOffset;
    }

//    private final double offset;
//    public final double orientation;
//    private final double angle;
//
//    /**
//     * Orientation: Camera pointing direction relative to straight ahead
//     * Angle: Angle of vector pointing from center of robot
//     */
//    public AprilTagCamera(WebcamName webcamName, double offset, double orientation, double angle) {
//        this.webcamName = webcamName;
//        this.offset = offset;
//        this.orientation = orientation;
//        this.angle = angle;
//    }
//
//    public Pose2d translatePoseOld(Pose2d inputPose) {
//        double thetaNeed = Math.toRadians(180) - angle - inputPose.getHeading() + orientation;
//        android.util.Log.e("APRILTAG", "MAGIC ANGLE: " + Math.toDegrees(thetaNeed));
//
//        double offsetX = offset * Math.cos(thetaNeed);
//        double offsetY = offset * Math.sin(thetaNeed);
//
//        android.util.Log.i("APRILTAG", "Camera offset: " + offsetX + ", " + offsetY);
//
//        double robotX = inputPose.getX() + offsetX;
//        double robotY = inputPose.getY() + offsetY;
//        double robotHeading = inputPose.getHeading() - orientation;
//
//        android.util.Log.e("APRILTAG", new Pose2d(robotX, robotY, robotHeading).toString());
//
//        return new Pose2d(robotX, robotY, robotHeading);
//    }

    /**
     * Incredibly nieve back-up implementation, translates exclusively based on quadrant
     * @param inputPose Pose2d to translate
     * @return translated Pose2d
     */
    public Pose2d translatePose(Pose2d inputPose) {
        boolean isRed = AprilTagLocalizer.whichQuadrant(inputPose) == AprilTagLocalizer.Quadrant.RED_AUDIENCE || AprilTagLocalizer.whichQuadrant(inputPose) == AprilTagLocalizer.Quadrant.RED_BOARD;
        return new Pose2d(inputPose.getX() + (isRed ? -sideOffset : sideOffset), inputPose.getY() + (isRed ? -forwardOffset : forwardOffset), isRed ? Math.toRadians(90) : Math.toRadians(270));
//        // 6.5: Distance from left camera to center of robot (now 6)
//        // -6.5: Distance from right camera to center of robot
//        // 5: distance from center of robot to cameras (forward) (now 9)
//        switch (AprilTagLocalizer.whichQuadrant(inputPose)) {
//            case RED_AUDIENCE:
//                return new Pose2d(inputPose.getX() + 6, inputPose.getY() - 9, Math.toRadians(90));
//            case RED_BOARD:
//                return new Pose2d(inputPose.getX() - 6, inputPose.getY() - 9, Math.toRadians(90));
//            case BLUE_AUDIENCE:
//                return new Pose2d(inputPose.getX() + 6, inputPose.getY() + 9, Math.toRadians(270));
//            case BLUE_BOARD:
//                return new Pose2d(inputPose.getX() - 6, inputPose.getY() + 9, Math.toRadians(270));
//        }
        //return null;
    }
}
