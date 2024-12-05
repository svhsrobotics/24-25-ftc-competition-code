package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class PsiParams extends SparkFunOTOSDrive.Params{


    public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
    public double lateralInPerTick =0.6384595875835196;
    public double trackWidthTicks = 0;


    // feedforward parameters (in tick units)
    public double kS = 0;
    public double kV = 0;
    public double kA = 0;


    // path profile parameters (in inches)
    public double maxWheelVel = 50;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = Math.PI; // shared with path
    public double maxAngAccel = Math.PI;

    // path controller gains
    public double axialGain = 0.0;
    public double lateralGain = 0.0;
    public double headingGain = 0.0; // shared with turn

    public double axialVelGain = 0.0;
    public double lateralVelGain = 0.0;
    public double headingVelGain = 0.0;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(-0.3965, 5.0883, Math.toRadians(-90.4721));

    public double linearScalar = 1.00403633333;
    public double angularScalar =1;


}
