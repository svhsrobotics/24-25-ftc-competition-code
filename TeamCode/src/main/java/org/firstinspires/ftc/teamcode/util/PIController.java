package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.util.Range;

public class PIController {
    double pGain;
    double iGain;
    double iWindupLimit;

    double iSum = 0;

    public PIController(double pGain, double iGain, double iWindupLimit) {
        this.pGain = pGain;
        this.iGain = iGain;
        this.iWindupLimit = iWindupLimit;
    }

    public double update(double input) {
        iSum += input;
        iSum = Range.clip(iSum, -iWindupLimit, iWindupLimit);

        return pGain * input + iGain * iSum;
    }
}
