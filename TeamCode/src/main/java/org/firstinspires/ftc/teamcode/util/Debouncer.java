package org.firstinspires.ftc.teamcode.util;

public class Debouncer {
    private boolean lastState = false;
    private  boolean isPressed = false;
    public boolean update(boolean buttonState) {

        if (buttonState != lastState) {

                isPressed = true;


        } else {
            isPressed = false;
        }
        lastState = buttonState;
        return isPressed;
    }
}

