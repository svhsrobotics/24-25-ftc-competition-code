package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.util.AimAssist;
@Config
@TeleOp(name="HIIIIIIIIIIIII")
public class SomethingMoreDistinct extends LinearOpMode {
     AimAssist gun = new AimAssist();
private double angle = 30;
private double xDistance = 130;
private double yDistance = 30.9;
private double power= 0;
//power should should be 50

    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        while(opModeIsActive()){
          power = gun.run(angle,xDistance, yDistance);
          telemetry.addData("result", power);
          telemetry.update();
          //todo fix it
            System.out.println(power);




        }

    }

}
