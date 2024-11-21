package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.AprilTagCamera;

public abstract class Robot {
    public abstract TrajectoryDrive getDrive();

    public static Robot thisRobot(HardwareMap hardwareMap) {
        // Set one of the Analog inputs to one of these names so we can tell which bot this is
        if (hardwareMap.tryGet(AnalogInput.class, "psi_bot") != null) {
            return new PsiBot(hardwareMap);
        } else if (hardwareMap.tryGet(AnalogInput.class, "robotica_bot") != null) {
            return new RoboticaBot(hardwareMap);
        } else if (hardwareMap.tryGet(AnalogInput.class, "test_bot") != null) {
            return new TestBot(hardwareMap);
        } else {
            return new TestBot(hardwareMap);
        }
    }

    protected Robot(HardwareMap hardwareMap) {
        // Any shared initialization goes here
    }

    public abstract AprilTagCamera[] getCameras();
    public abstract AprilTagCamera getPrimaryCamera();

    /**
     * Sets the purple pixel on the robot;
     * true: dropped;
     * false: closed
     */
    public abstract void dropPurplePixel(boolean state);

    public abstract void launchPlane();
}
