package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ColorBenchTest extends OpMode {
    ColorTest bench = new ColorTest();


    @Override
    public void init() {
        bench.init(hardwareMap);
    }

    @Override
    public void loop () {
        bench.getDetectedColor(telemetry);
    }
}
