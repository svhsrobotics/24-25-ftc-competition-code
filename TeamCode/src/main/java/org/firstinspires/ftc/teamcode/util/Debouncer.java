package org.firstinspires.ftc.teamcode.util;

public class Debouncer {
    public boolean lastState = false;

    public boolean update(boolean buttonState) {
        if (buttonState) {
            if (!lastState) {
                lastState = true;
            }
        } else {
            lastState = false;
        }
        return lastState;
    }
}
