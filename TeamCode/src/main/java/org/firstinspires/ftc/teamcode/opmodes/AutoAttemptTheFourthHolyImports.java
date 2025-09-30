package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.SparkFunOTOSDrive.NewDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
@Config
@Autonomous
public class AutoAttemptTheFourthHolyImports extends LinearOpMode {


    private class LauncherClass {

        //Mechanical Components (SPECIFICALLY NOT OBJECTS)
        private CRServo motor1;
        private DcMotor motor2;


        public LauncherClass (HardwareMap hardwareMap){
            motor1 = hardwareMap.get(CRServo.class, "motor1");

           // motor2 = hardwareMap.get(DcMotor.class, "motor2");
        }
        public class MaxSpeedLaunchClass implements Action{
            private boolean init = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!init){
                    motor1.setPower(1);

                   // motor2.setPower(1);
            }
                return false;

        }




     }
     public Action maxLaucherSpeed(){
            return new MaxSpeedLaunchClass();
        }
}

    @Override
    public void runOpMode() throws InterruptedException {
        LauncherClass launcher = new LauncherClass(hardwareMap);
        Pose2d startPose = new Pose2d(0, 0, 0);
        SparkFunOTOSDrive drive = NewDrive(hardwareMap, startPose);
        TrajectoryActionBuilder steve = drive.actionBuilder(startPose)
                .strafeTo(new Vector2d(-30, 30));
        Action trajectoryActionCloseout = steve.endTrajectory().fresh().build();
        waitForStart();
        if(isStopRequested()) return;
        Actions.runBlocking(
                new SequentialAction(steve.build(),
                        launcher.maxLaucherSpeed(),
                        trajectoryActionCloseout));
    }


}
