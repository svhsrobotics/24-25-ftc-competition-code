package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class PsiParams extends SparkFunOTOSDrive.Params{
 public PsiParams(HardwareMap hardwareMap){
     leftFront= hardwareMap.get(DcMotorEx.class, "leftFront");
     rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
     leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
     rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
     rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
     leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
     leftBack.setDirection(DcMotorSimple.Direction.REVERSE);


 }

    public RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection =
            RevHubOrientationOnRobot.LogoFacingDirection.RIGHT;

    public RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection =
            RevHubOrientationOnRobot.UsbFacingDirection.UP;

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

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0.4806, 61.3596, -175.3088);

    public double linearScalar = 100/100.1193;
    public double angularScalar =1.0023;

    @Override
    public RevHubOrientationOnRobot.LogoFacingDirection getLogoFacingDirection() {
        return logoFacingDirection;
    }
public DcMotorEx leftFrontMotor, rightFrontMotor, leftBackMotor, rightBackMotor;
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
