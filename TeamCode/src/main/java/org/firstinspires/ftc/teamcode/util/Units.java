package org.firstinspires.ftc.teamcode.util;

public class Units {
    public static int fi(int feet, int inches) {
        if (feet < 0 && inches > 0) {
            inches = inches * -1;
        }
        return feet * 12 + inches;
    }
}
