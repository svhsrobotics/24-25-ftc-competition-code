package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;

public class OmegaParams extends SparkFunOTOSDrive.Params{



    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // drive model parameters
    public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
    public double lateralInPerTick = 0.8689404978341202;
    public double trackWidthTicks = 10.145082137841747;


    // feedforward parameters (in tick units)
    public double kS = 0.24188825066206476;
    public double kV = 0.679465173936135;
    public double kA = 0.135;


    // path profile parameters (in inches)
    public double maxWheelVel = 25;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = Math.PI; // shared with path
    public double maxAngAccel = Math.PI;

    // path controller gains
    public double axialGain = 3.5;
    public double lateralGain = 3.5;
    public double headingGain = 3; // shared with turn

    public double axialVelGain = 1;
    public double lateralVelGain = 1;
    public double headingVelGain = 1.0;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(7.8637, 0.2763, -1.5838);




    public double linearScalar = 100/102.8887;
    public double angularScalar = .9926;
}
