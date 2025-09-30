package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorTest {
    RevColorSensorV3 colorSensor;

    public enum DetectedColor {
        GREEN,
        BLUE,
        RED,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(RevColorSensorV3.class, "Color Cam");
    }

    public DetectedColor getDetectedColor (Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors(); // returns four values
        float normBlue, normGreen, normRed;
        normBlue = colors.blue / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normRed = colors.red / colors.alpha;

        telemetry.addData("blue", normBlue);
        telemetry.addData("green", normGreen);
        telemetry.addData("red", normRed);

        //to do: add if statements for specific colors
        /*
        RED =
        GREEN =
        BLUE =
         */


        return DetectedColor.UNKNOWN;
    }


}
