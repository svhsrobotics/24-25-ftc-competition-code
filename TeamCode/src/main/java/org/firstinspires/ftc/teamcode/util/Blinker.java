package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Blinker.Step;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.TimeUnit;

/**
 * Control the status LED built-in to REV Control Hubs and REV Expansion Hubs
 */
public class Blinker {
    private final LynxModule module;

    /**
     * Create a Blinker wrapper
     * @param module the REV Hub to use
     */
    public Blinker(LynxModule module) {
        this.module = module;
    }

    /**
     * @param pattern the pattern to display
     * @see Pattern
     */
    public void displayPattern(Pattern pattern) {
        module.setPattern(pattern.steps);
    }

    /**
     * Display the idle pattern
     * <br/>
     * <ul>
     *     <li>Expansion Hub</li>
     *     <ul>
     *         <li>GREEN for 5 seconds</li>
     *         <li>OFF for .5 seconds</li>
     *         <li>BLUE flashs corresponding to the ID (.5 ea. followed by .5 OFF)</li>
     *     </ul>
     *     <li>Control Hub: GREEN</li>
     * </ul>
     */
    public void idle() {
        module.setPattern(new LynxModule.CountModuleAddressBlinkerPolicy().getIdlePattern(module));
    }

    /**
     * Display a pattern that makes the module easily identifiable
     * <br/>
     * <ul>
     *     <li>CYAN for 150ms</li>
     *     <li>OFF for 75ms</li>
     *     <li>MAGENTA for 150ms</li>
     *     <li>OFF for 75ms</li>
     * </ul>
     */
    public void identify() {
        module.setPattern(new LynxModule.BreathingBlinkerPolicy().getVisuallyIdentifyPattern(module));
    }

    public static class Pattern {
        public static final int MAX_STEPS = 16;

        private final List<Step> steps = new ArrayList<>();

        /**
         * Add a step to the pattern. You can have a maximum of 16 steps.
         * @param color the color to display (in Android's int color format)
         * @param duration the duration in ms for this step to last
         * @see android.graphics.Color
         */
        public Pattern addStep(int color, int duration) {
            steps.add(new Step(color, duration, TimeUnit.MILLISECONDS));
            return this;
        }
    }
}
