package org.firstinspires.ftc.teamcode.opmodes;
import org.firstinspires.ftc.teamcode.util.Toggle;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
@Config
public class intakeClawMechanismTest extends LinearOpMode {
private final Servo intakeClawServo = hardwareMap.get(Servo.class, "intakeClawServo");
private final Servo intakeRotationServo = hardwareMap.get(Servo.class, "intakeRotationServo");
private final Toggle intakeToggle = new Toggle();
    @Override
    public void runOpMode(){

       waitForStart();
       if(gamepad1.dpad_up){
           intakeRotationServo.setPosition(intakeRotationServo.getPosition() + 0.01);

       }
       else if(gamepad1.dpad_down){
           intakeRotationServo.setPosition((intakeRotationServo.getPosition() -0.01));
       }
       intakeToggle.update(gamepad1.a);
       if(!intakeToggle.state){
           intakeClawServo.setPosition(1);
       }
       else{
           intakeClawServo.setPosition(0);
       }




    }
}
