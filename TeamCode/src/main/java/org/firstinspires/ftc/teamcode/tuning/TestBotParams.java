package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;

@Config
public class TestBotParams extends SparkFunOTOSDrive.Params{
    public TestBotParams(HardwareMap hardwareMap) {
        /*leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");*/
        //robot was backwards i think
        leftFront = hardwareMap.get(DcMotorEx.class, "rightBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "leftBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "rightFront");
        rightBack = hardwareMap.get(DcMotorEx.class, "leftFront");

    }
    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.UP;
    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

    // drive model parameters


    public  double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
    public  double lateralInPerTick = 0.7851802527161125; //0.736834497757;// OTOS: 0.872882;
    public  double trackWidthTicks = 9.934084835760123; //12.791; // otos 12.66;

    // feedforward parameters (in tick units)

    public   double kS = 0.6127580753029531; //0.7635681070147831; // OTOS: 0.563756515907424;
    public  double kV = 0.22577022205832956; //0.1946438443334511; // OTOS:0.19141851548064043;
    public  double kA = 1;

    // path profile parameters (in inches)
    public  double maxWheelVel = 60; // 50
    public  double minProfileAccel = -30;
    public  double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public  double maxAngVel = Math.PI; // shared with path
    public  double maxAngAccel = Math.PI;

    // path controller gains
    public  double axialGain = 8.0;
    public  double lateralGain = 8.0;
    public  double headingGain = 8.0; // shared with turn

    public  double axialVelGain = 0.0;
    public  double lateralVelGain = 0.0;
    public  double headingVelGain = 0.0; // shared with turn

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(3.4362, -0.1201, 1.6018);


    public  double linearScalar =48/35.9723 ;
    public  double angularScalar = 0.9965;

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

