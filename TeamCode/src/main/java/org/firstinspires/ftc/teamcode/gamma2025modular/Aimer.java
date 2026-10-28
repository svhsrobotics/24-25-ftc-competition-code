package org.firstinspires.ftc.teamcode.gamma2025modular;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Aimer: the webcam, finding the goal's AprilTag, and working out how to turn toward it.
 *
 * WHAT: Remembers which goal we're aiming at (blue or red), looks for that goal's tag each
 *       pass, and answers: can we see it, how far is it, are we lined up ("locked on"), and
 *       how hard should the robot turn to line up.
 * FROM: BobComp lines 41, 48 to 55 (fields), 77 to 89 (camera setup), 110 to 112 (start values),
 *       200 to 206 (swap), 227 to 239 (reading tags), 264 to 266 (locked check),
 *       315 (turn power).
 *
 * DECODE tags: 20 = blue goal, 24 = red goal. (BIOBUZZ uses tags 30 to 45, under the HIVE CELLs.)
 */
public class Aimer {

    public static final int BLUE_GOAL_TAG = 20;
    public static final int RED_GOAL_TAG = 24;

    // Auto-aim points the robot this many degrees off the camera's center, probably because the
    // camera isn't exactly in line with the launcher. Found by testing on Bob.
    private static final double AIM_POINT_DEGREES = 0.5;

    // How hard to turn per degree off target (BobComp: "sensitivity"). 10 degrees off = 0.29 power.
    private static final double SENSITIVITY = 0.03;

    private AprilTagProcessor tagProcessor;   // finds tags in each camera image
    private VisionPortal visionPortal;        // the camera pipeline that feeds it images

    private int targetID = BLUE_GOAL_TAG;     // BobComp started on blue (line 111)
    private boolean targetSeen = false;
    private double bearing = 0;               // BobComp: targetHeading. Degrees off center
    private double distance = 0;              // centimeters

    // Start the camera. Called once by BobRobot.init().
    public void init(HardwareMap hardwareMap) {
        // "Builder" pattern: choose options one at a time, then build() makes the object.
        // The four setDraw lines only draw on the camera preview; they don't change detection.
        // setOutputUnits(CM, DEGREES) is why every distance in this code is in centimeters.
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();

        // Which camera ("Webcam 1" is its configuration name), picture size, which processor.
        // build() turns the camera on; from then on it processes frames in the background.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(tagProcessor);
        visionPortal = builder.build();
    }

    // ---- which goal --------------------------------------------------------------------

    // Swap between the blue and red goal.
    public void swapTarget() {
        if (targetID == BLUE_GOAL_TAG) {
            targetID = RED_GOAL_TAG;
        } else {
            targetID = BLUE_GOAL_TAG;
        }
    }

    public boolean isTargetingBlue() {
        return targetID == BLUE_GOAL_TAG;
    }

    // ---- looking for the tag -----------------------------------------------------------

    /*
     * Look for our goal's tag in the camera's latest results. Call once per loop pass.
     *
     * Starts by assuming "not seen" and only sets it true if the tag is in the list.
     * IMPORTANT: when the tag isn't seen, bearing and distance KEEP THEIR LAST VALUES.
     * Always check canSeeTarget() before trusting them.
     */
    public void update() {
        targetSeen = false;
        List<AprilTagDetection> detections = tagProcessor.getDetections();

        // "For each tag found in this frame, call it detection..."
        for (AprilTagDetection detection : detections) {
            // metadata is what the SDK knows about a tag from the game's tag list. A tag that
            // isn't on the list has no metadata, and reading .id from null would crash.
            // && checks left to right and stops at the first false, so the null check MUST
            // come first. ("Short-circuit".)
            if (detection.metadata != null
                    && detection.metadata.id == targetID) {
                bearing = detection.ftcPose.bearing;   // degrees off the center of the image
                distance = detection.ftcPose.range;    // straight-line distance, in cm
                targetSeen = true;
            }
        }
    }

    public boolean canSeeTarget() {
        return targetSeen;
    }

    public double getBearing() {
        return bearing;
    }

    public double getDistance() {
        return distance;
    }

    // ---- lining up ---------------------------------------------------------------------

    /*
     * Lined up well enough to shoot: the tag is seen AND the bearing is between -1.5 and 2.5
     * degrees. That window is centered on the 0.5 degree aim point, so "locked on" means within
     * 2 degrees of where auto-aim is pointing.
     * Why "targetSeen" matters: without it, an old bearing from seconds ago could say "locked".
     */
    public boolean isLockedOn() {
        return bearing < 2.5
                && bearing > -1.5
                && targetSeen;
    }

    /*
     * How hard to turn to line up (proportional control): the farther off, the harder it
     * turns, so it slows down as it lines up and doesn't overshoot. A dimmer, not a light switch.
     * The Drivetrain does the actual turning: robot.drive.turnInPlace(robot.aimer.turnPower()).
     */
    public double turnPower() {
        return (bearing - AIM_POINT_DEGREES) * SENSITIVITY;
    }
}
