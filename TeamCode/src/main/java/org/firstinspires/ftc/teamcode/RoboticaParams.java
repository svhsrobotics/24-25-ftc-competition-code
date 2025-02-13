package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class RoboticaParams extends SparkFunOTOSDrive.Params{
    public RoboticaParams(HardwareMap hardwareMap) {
        // TODO: Get motors from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "left_front");
        rightFront = hardwareMap.get(DcMotorEx.class, "right_front");
        leftBack = hardwareMap.get(DcMotorEx.class, "left_back");
        rightBack = hardwareMap.get(DcMotorEx.class, "right_back");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1
    public double lateralInPerTick =0.7164157253262274;
    public double trackWidthTicks = 12.246237486392962;


    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.DOWN;


    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.RIGHT;

    // drive model parameters



    // feedforward parameters (in tick units)
    public static double kS = 0.6767740353758827;
    public static double kV = 0.277894824985166;
    public static double kA = 0.01;


    // path profile parameters (in inches)
    public double maxWheelVel = 50;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public double maxAngVel = Math.PI; // shared with path
    public double maxAngAccel = Math.PI;

    // path controller gains
    public static double axialGain = 5;
    public static double lateralGain = 5;
    public static double headingGain = 5; // shared with turn

    public static double axialVelGain = 1;
    public static double lateralVelGain = 1;
    public static double headingVelGain = 1;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public static double yoffset = 5.9349;

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(.6308, yoffset, 0.0104);


    public double linearScalar = 90/90.976;
    public double angularScalar = 1.0138;

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
