package org.firstinspires.ftc.teamcode.gamma2025modular;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

/*
 * BobRobot: the whole robot in one object.
 *
 * WHAT: Builds every part of Bob (drivetrain, launcher, intake, aimer) from the robot's
 *       hardware configuration. Any OpMode does "robot.init(hardwareMap)" once, then uses
 *       robot.drive, robot.launcher, robot.intake and robot.aimer.
 *
 * WHY:  In BobComp, about 40 lines of setup (lines 60 to 99) were copied into every OpMode,
 *       and the copies drifted apart (rightPush is FORWARD in BobComp but REVERSE in
 *       BobAutoLEFT). With one hardware class, the setup exists once, so it can't drift.
 *       This is the pattern FIRST recommends: see the SDK samples
 *       ConceptExternalHardwareClass.java and RobotHardware.java.
 *
 * Each part lives in its own file and finds its own motors and servos. This class just
 * builds them all. For BIOBUZZ, the same idea becomes GammaRobot.java.
 */
public class BobRobot {

    // The four parts. "public" so an OpMode can use them: robot.drive.drive(...)
    // "new" makes each part right away; init() below connects it to the real hardware.
    public Drivetrain drive = new Drivetrain();
    public Launcher launcher = new Launcher();
    public Intake intake = new Intake();
    public Aimer aimer = new Aimer();

    // Set up in BobComp but never read. Kept for later ideas like field-centric driving.
    // (Bob also found the battery voltage sensor but never used it, so it's left out.)
    public IMU imu;

    // Call once, from the OpMode's init(). Nothing moves here: it only finds and sets up devices.
    public void init(HardwareMap hardwareMap) {
        drive.init(hardwareMap);
        launcher.init(hardwareMap);
        intake.init(hardwareMap);
        aimer.init(hardwareMap);      // the aimer's init also turns the camera on

        // The IMU only knows how the HUB is turned. These two settings say how the hub is
        // mounted on Bob (REV logo up, USB ports facing the robot's right), so it can work
        // out which way the ROBOT faces. A new robot with a different hub mounting needs new values.
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot orientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.RIGHT
        );
        imu.initialize(new IMU.Parameters(orientation));
    }
}
