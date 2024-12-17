package org.firstinspires.ftc.teamcode.util;

/***
 * PID controller
 * create a new variable from this class
 * then call it while passing in the reference Ki Kp and Kd
 * ex. PID = new PIDController2(reference, Ki, Kp, Kd)
 * then set your motor power to your new variable while passing in the encoder position
 * ex. motor.setPower(PID.update(motor.getCurrentPosition())
 */
public class PIDController2 {
    public double reference = 0;
    public double Kp=0;
   public double prevError = 0;
    double iError = 0;
   double Ki=0;
   double Kd=0;
   double pidLimit=1;


    /**
     *
     * @param newReference the reference that gets passed in from wherever you call this from
     * @param newKi the constant for the integer part of the final equation
     * @param newKd the constant for the derivative part
     * @param newKp the constant for the Proportional part
     */
   public PIDController2(double newReference, double newKi, double newKp, double newKd){
       reference = newReference;
       Kp = newKp;
       prevError = 0;
       Kd = newKd;
       Ki = newKi;

       //double error = 79;

   }






    /*** run theloop
     *
     * @param encoderPositionA this is the input to the PID loop
     * @return resutling output from PID loop
     */
    public double update(double encoderPositionA) {

        // obtain the encoder position
        double error;
        double diff;
        double power;


        error = reference - encoderPositionA;

        iError = iError + error;

        //cumulative error
        diff = error - prevError;

        //calculate the D
        prevError = error;

        // calculate the error
        //power = Kp * (error) + Ki * (iError) + Kd * (diff);
        power = Kp * (error);
        if(power > pidLimit) {
            power = pidLimit;
        }
        else if(power< -pidLimit){
            power = -pidLimit;
        }

        return power;
    }

}
