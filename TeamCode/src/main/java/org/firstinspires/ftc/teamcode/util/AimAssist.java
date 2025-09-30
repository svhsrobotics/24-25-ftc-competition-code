package org.firstinspires.ftc.teamcode.util;

/***
 * angle: the angle the gun is aimed at
 * xDistance: the horizontal distance from the robot to the target
 * yDistance: the vertical distance between the robot and the target
 * wheelRadius: the radius of the wheel that turns
 * radsPerSec: Self Explanatory
 */
public class AimAssist {

    public double run(double angle, double xDistance, double yDistance, double wheelRadius ) {
        double power =
                Math.sqrt(
                        4.9*Math.pow(xDistance/Math.cos(angle), 2)
                        /
                                xDistance * Math.tan(angle) - yDistance

                );
        double realpower = power/wheelRadius;
/*v = rw
v/r = w

 */
        return(realpower);


    }

}
