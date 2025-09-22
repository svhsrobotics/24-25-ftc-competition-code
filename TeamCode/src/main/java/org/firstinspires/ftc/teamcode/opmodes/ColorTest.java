package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorTest {
    RevColorSensorV3 colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        RED,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(RevColorSensorV3.class, "Color Cam");
    }

    public DetectedColor getDetectedColor (Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns four values
        float normPurple, normGreen, normRed;
        normPurple = colors.blue / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normRed = colors.red / colors.alpha;

        telemetry.addData("purple", normPurple);
        telemetry.addData("green", normGreen);
        telemetry.addData("red", normRed);

        //to do: add if statements for specific colors
        /*
        RED =
        GREEN =
        PURPLE =
         */


        return DetectedColor.UNKNOWN;
    }


}
