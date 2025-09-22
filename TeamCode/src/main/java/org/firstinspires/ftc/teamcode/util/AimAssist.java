package org.firstinspires.ftc.teamcode.util;

/***
 * angle: the angle the gun is aimed at
 * xDistance: the horizontal distance from the robot to the target
 * yDistance: the vertical distance between the robot and the target
 */
public class AimAssist {
    public  AimAssist(){}
    public double run(double angle, double xDistance, double yDistance ) {
        double power =
                Math.sqrt(
                        4.9*Math.pow(xDistance/Math.cos(angle), 2)
                        /
                                xDistance * Math.tan(angle) - yDistance

                )


                ;

        return(power);


    }

}
