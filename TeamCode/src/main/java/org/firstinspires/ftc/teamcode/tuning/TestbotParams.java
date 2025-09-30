package org.firstinspires.ftc.teamcode.tuning;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;


public class TestbotParams extends SparkFunOTOSDrive.Params {
    public TestbotParams(HardwareMap hardwareMap) {
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
    }

    public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
    public double lateralInPerTick =0.6384595875835196;
    public double trackWidthTicks = 0;


    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;


    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

    // drive model parameters



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

    @Override
    public RevHubOrientationOnRobot.LogoFacingDirection getLogoFacingDirection() {
        return logoFacingDirection;
    }

    @Override
    public RevHubOrientationOnRobot.UsbFacingDirection getUsbFacingDirection() {
        return usbFacingDirection;
    }

    @Override
    public double getInPerTick() {
        return inPerTick;
    }

    @Override
    public double getLateralInPerTick() {
        return lateralInPerTick;
    }

    @Override
    public double getTrackWidthTicks() {
        return trackWidthTicks;
    }

    @Override
    public double getKS() {
        return kS;
    }

    @Override
    public double getKV() {
        return kV;
    }

    @Override
    public double getKA() {
        return kA;
    }

    @Override
    public double getMaxWheelVel() {
        return maxWheelVel;
    }

    @Override
    public double getMinProfileAccel() {
        return minProfileAccel;
    }

    @Override
    public double getMaxProfileAccel() {
        return maxProfileAccel;
    }

    @Override
    public double getMaxAngVel() {
        return maxAngVel;
    }

    @Override
    public double getMaxAngAccel() {
        return maxAngAccel;
    }

    @Override
    public double getAxialGain() {
        return axialGain;
    }

    @Override
    public double getLateralGain() {
        return lateralGain;
    }

    @Override
    public double getHeadingGain() {
        return headingGain;
    }

    @Override
    public double getAxialVelGain() {
        return axialVelGain;
    }

    @Override
    public double getLateralVelGain() {
        return lateralVelGain;
    }

    @Override
    public double getHeadingVelGain() {
        return headingVelGain;
    }

    @Override
    public DcMotorEx getLeftFront() {
        return leftFront;
    }

    @Override
    public DcMotorEx getLeftBack() {
        return leftBack;
    }

    @Override
    public DcMotorEx getRightBack() {
        return rightBack;
    }

    @Override
    public DcMotorEx getRightFront() {
        return rightFront;
    }

    @Override
    public SparkFunOTOS.Pose2D getOffset() {
        return offset;
    }

    @Override
    public double getLinearScalar() {
        return linearScalar;
    }

    @Override
    public double getAngularScalar() {
        return angularScalar;
    }
}
