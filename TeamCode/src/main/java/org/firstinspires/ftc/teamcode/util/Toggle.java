package org.firstinspires.ftc.teamcode.util;

/**
 * This is a simple class that allows you to implement a toggle button.
 * Simply call update() with the button's current value and it will return the toggle's state.
 * You can also read and set state directly to override it.
 */
public class Toggle {
    public boolean state = false;
    private boolean lastState = false;

    public boolean update(boolean buttonState) {
        // If the button is pressed
        if (buttonState) {
            // If the button's last state was "not pressed"
            if (!lastState) {
                // Toggle our current state
                state = !state;
            }
            // The button was last pressed
            lastState = true;
        // The button is not pressed
        } else {
            // The button was last not pressed
            lastState = false;
        }

        return state;
    }
}
