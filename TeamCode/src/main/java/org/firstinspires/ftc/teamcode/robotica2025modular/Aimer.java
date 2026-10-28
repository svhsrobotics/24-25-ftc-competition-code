package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

/*
 * Aimer: the webcam, finding an AprilTag, and working out how to turn to center it.
 *
 * WHAT: Each pass, looks for a tag and remembers where its center is across the camera
 *       picture (in pixels). Answers: can we see one, where is it, are we centered, and how
 *       hard should the robot turn to center it.
 * FROM: DecodeTestTeleOp lines 42 to 45 (fields), 102 to 105 and 111 to 165 (camera setup;
 *       mostly FIRST's sample comments), 254 to 259 (camera on/off), 265 to 270 (reading the
 *       tag), 272 to 287 (centering numbers: 290, 310, 0.25).
 *
 * NOTE (same as Robotica): it uses the FIRST tag in the list, whichever tag that is. In
 * DECODE that could be a goal tag or one of the tags on the obelisk. Gamma's Aimer only
 * accepts its goal's tag number.
 */
public class Aimer {

    // The tag counts as centered when its center is between these pixel columns.
    // The picture is probably 640 pixels wide, so the middle is 320: these numbers center a
    // little left of the middle, maybe because the camera isn't exactly in line with the
    // launcher. Verify with the team.
    private static final double CENTER_LEFT_EDGE = 290;
    private static final double CENTER_RIGHT_EDGE = 310;
    private static final double TURN_POWER = 0.25;

    private AprilTagProcessor aprilTag;   // finds tags in each camera image
    private VisionPortal visionPortal;    // the camera pipeline that feeds it images

    private boolean targetSeen = false;
    private double tagCenterX = 0;        // pixels from the left edge of the picture

    // Start the camera.
    // NOTE (same as Robotica): RoboticaTeleOp calls this AFTER START (see there).
    public void init(HardwareMap hardwareMap) {
        // Where the camera sits on the robot: 8 inches forward of the robot's center, tilted
        // (pitch -90 + 19 degrees). Measured by Robotica. (Same numbers as Psi's camera.)
        Position cameraPosition = new Position(DistanceUnit.INCH,
                0, 8, 0, 0);
        YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90 + 19, 0, 0);

        // "Builder" pattern: choose options one at a time, then build() makes the object.
        aprilTag = new AprilTagProcessor.Builder()
                .setCameraPose(cameraPosition, cameraOrientation)
                .build();

        // Which camera ("Webcam 1" is its configuration name) and which processor.
        // build() turns the camera on; from then on it processes frames in the background.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();
    }

    // ---- looking for a tag -------------------------------------------------------------

    /*
     * Look at the camera's latest results. Call once per loop pass.
     * IMPORTANT: when no tag is seen, tagCenterX KEEPS ITS LAST VALUE.
     * Always check canSeeTarget() before trusting it.
     * (Robotica stored the tag's center in x, the same variable that held the strafe stick
     * (lines 194 and 269). One name, two jobs. A separate name here makes that mix-up
     * impossible.)
     */
    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();
        targetSeen = !detections.isEmpty();
        if (targetSeen) {
            tagCenterX = detections.get(0).center.x;
        }
    }

    public boolean canSeeTarget() {
        return targetSeen;
    }

    public double getTagCenterX() {
        return tagCenterX;
    }

    // ---- centering ---------------------------------------------------------------------

    // A tag is in view and its center is between 290 and 310 pixels.
    public boolean isCentered() {
        return targetSeen
                && tagCenterX >= CENTER_LEFT_EDGE
                && tagCenterX <= CENTER_RIGHT_EDGE;
    }

    /*
     * How hard to turn to center the tag: 0.25 one way or the other, or 0 when it's centered
     * or no tag is in view. + = turn right. The Drivetrain does the actual turning:
     * robot.drive.turnInPlace(robot.aimer.turnPower()).
     * Tag right of center (over 310): turn right. Left of center (under 290): turn left.
     * That's the same direction Robotica's loop turned (it powered only the left wheels to
     * turn right, only the right wheels to turn left).
     * Unlike Gamma's turnPower(), this is a light switch, not a dimmer: always 0.25, no matter
     * how far off. Gamma's turns harder the farther off it is, so it slows down as it lines up.
     */
    public double turnPower() {
        if (!targetSeen || isCentered()) {
            return 0;
        }
        if (tagCenterX > CENTER_RIGHT_EDGE) {
            return TURN_POWER;
        }
        return -TURN_POWER;
    }

    // ---- camera on/off (D-pad up) ------------------------------------------------------

    /*
     * Meant to switch the tag reader on or off.
     * NOTE (same as Robotica, lines 254 to 259): trace it one line at a time. If the reader is
     * off, the first "if" turns it on, and then the second "if" sees it's on and turns it off
     * again. If it's on, the second "if" turns it off. Either way it ends OFF, so D-pad up
     * really means "tag reader off", and nothing turns it back on.
     * The fix: one if/else (if on, turn off; else turn on), plus a "once per press" check in
     * the TeleOp so holding the button doesn't flip it 50 times a second.
     */
    public void toggleCamera() {
        if (!visionPortal.getProcessorEnabled(aprilTag)) {
            visionPortal.setProcessorEnabled(aprilTag, true);
        }
        if (visionPortal.getProcessorEnabled(aprilTag)) {
            visionPortal.setProcessorEnabled(aprilTag, false);
        }
    }
}
