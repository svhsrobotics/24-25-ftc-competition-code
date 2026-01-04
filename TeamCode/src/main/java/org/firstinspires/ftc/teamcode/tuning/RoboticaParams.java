package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.tuning.otos.SparkFunOTOSDrive;

@Config
public class RoboticaParams extends SparkFunOTOSDrive.Params {
    public RoboticaParams(HardwareMap hardwareMap) {
        // TODO: Get motors from hardware map
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");

        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBack.setDirection(DcMotorSimple.Direction.REVERSE);



    }

    public double inPerTick = 1; // SparkFun OTOS Note: you can probably leave this at 1g
    public double lateralInPerTick =  .872882;  // OTOS: 0.872882;
    public double trackWidthTicks = 12.66; // otos 12.66;



    public IMU.Parameters myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
          new Orientation(
                    AxesReference.INTRINSIC,
          AxesOrder.ZYX,
          AngleUnit.DEGREES,
               180,
                  0,
                  50,
                  0  // acquisitionTime, not used
    )
     )
             );





// drive model parameters



    // feedforward parameters (in tick units)
    public double kS = 0.563756515907424; //0.7635681070147831; // OTOS: 0.563756515907424;
    public double kV = 0.19141851548064043; //0.1946438443334511; // OTOS:0.19141851548064043;
    public static double kA = 0.01;

    public IMU.Parameters getMyIMUparameters() {
        return myIMUparameters;
    }


    // path profile parameters (in inches)
    public double maxWheelVel = 60;
    public double minProfileAccel = -30;
    public double maxProfileAccel = 50;

    // turn profile parameters (in radians)
    public static double maxAngVel = Math.PI; // shared with path
    public static double maxAngAccel = Math.PI;

    // path controller gains
    public static double axialGain = 8.0;
    public static double lateralGain = 8.0;
    public static double headingGain = 8.0; // shared with turn

    public static double axialVelGain = 0;
    public static double lateralVelGain = 0;
    public static double headingVelGain = 0;

    public DcMotorEx leftFront, leftBack, rightBack, rightFront;

    public SparkFunOTOS.Pose2D offset = new SparkFunOTOS.Pose2D(0, 0, -0.0064);


    public double linearScalar = 1;
    public double angularScalar = 1;

    @Override
    public IMU.Parameters getImuFacingDirection() {
        return myIMUparameters;
    }

//    @Override
//    public RevHubOrientationOnRobot.UsbFacingDirection getUsbFacingDirection() {
//        return usbFacingDirection;
//    }

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
