package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * RoboticaRobot: the whole robot in one object, plus the moves that use several parts at once.
 *
 * WHAT: Builds Robotica's parts from the hardware configuration. Any OpMode does
 *       "robot.init(hardwareMap)" once, then uses robot.drive, robot.launcher, robot.intake
 *       and robot.aimer. Moves that use several parts (get ready to shoot, start intaking,
 *       back up, center on the tag) live here, one level above the parts.
 * FROM: DecodeTestTeleOp lines 61 to 100 (setup), 204 to 227 (A, left trigger, X),
 *       245 to 253 (back up), 265 to 287 (reading the tag and centering).
 *
 * This follows FIRST's pattern for a hardware class: see the SDK samples
 * ConceptExternalHardwareClass.java and RobotHardware.java.
 *
 * Everything behaves like DecodeTestTeleOp, including its oddities, marked
 * "NOTE (same as Robotica)", except centerOnTag() (see there; decided by Mark, 2026-10-26).
 */
public class RoboticaRobot {

    // The four parts. "public" so an OpMode can use them: robot.drive.drive(...)
    // "new" makes each part right away; init() connects it to the real hardware.
    public Drivetrain drive = new Drivetrain();
    public Launcher launcher = new Launcher();
    public Intake intake = new Intake();
    public Aimer aimer = new Aimer();

    /*
     * Call once, from the OpMode before START.
     * NOTE (same as Robotica): the camera is NOT started here. RoboticaTeleOp starts it after
     * START, the way the old file did (see there).
     * NOTE (same as Robotica, lines 99 to 100): the triggers move during INIT. This season,
     * rule G403 says that if INIT moves anything, the team must wait until TELEOP starts before
     * pressing INIT. Could the triggers move right after START instead?
     */
    public void init(HardwareMap hardwareMap) {
        drive.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
        launcher.rest();   // triggers to their starting spot
    }

    // A, and the left trigger: spin up the flywheels and stop the intake.
    public void getReadyToShoot(double speed) {
        launcher.turnOn(speed);
        intake.stop();
    }

    /*
     * X: intake on, triggers back, flywheels off (lines 221 to 227).
     * NOTE (same as Robotica, with one tiny change): line 224 put the right trigger at 0.52,
     * but every other "rest" (lines 100 and 243) uses 0.515. Here X uses rest(), so it's 0.515
     * too, a difference of half a hundredth of the servo's travel (under 1 degree). Ask the
     * team: was 0.52 on purpose, or a typo? With one rest() method, there's one number to fix.
     */
    public void startIntaking() {
        intake.run(1);
        launcher.rest();
        launcher.turnOff();
    }

    /*
     * D-pad right: full power backward.
     * NOTE (same as Robotica, lines 245 to 253): the old code set all four motors to -1 inside a
     * loop that repeats 20,000 times. That does the same as setting them once: the SDK only
     * sends a power to the motor when it CHANGES (checked in the SDK 11 code), so the other
     * 79,996 calls do nothing. The loop doesn't make the back-up last longer. The robot backs up
     * while D-pad right is held, because on each pass this runs after the sticks have set the
     * drive. When it's let go, the sticks take over again on the next pass.
     */
    public void backUp() {
        drive.drive(-1, 0, 0);
    }

    /*
     * D-pad down (while not centered): turn toward the tag.
     *
     * FIXED (Mark, 2026-10-26). The old version (lines 272 to 287) was a while loop that kept
     * going until the tag was centered or D-pad down was let go. While it ran:
     *   - no other button or stick worked, and the Driver Station screen stopped updating;
     *   - it never checked opModeIsActive(), so pressing STOP didn't stop it;
     *   - if no tag was in view it used the strafe stick's number as the tag position (they
     *     shared the variable x), so the robot kept turning without ever finding the tag.
     * Now it's one small turn per loop pass, and the loop keeps running everything else.
     * The aimer works out how hard to turn; the drivetrain turns all four wheels.
     * (The old loop powered only one side, at 0.25, and left the other side at whatever the
     * sticks last set. Turning in place with all four wheels is steadier.)
     */
    public void centerOnTag() {
        drive.turnInPlace(aimer.turnPower());
    }

    // Call once per loop pass: read the camera.
    public void update() {
        aimer.update();
    }
}
