package org.firstinspires.ftc.teamcode.tuning;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.teamcode.OmegaParams;
import org.firstinspires.ftc.teamcode.PsiParams;
import org.firstinspires.ftc.teamcode.RoboticaParams;
import org.firstinspires.ftc.teamcode.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.TankDrive;

public final class SplineTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d beginPose = new Pose2d(0, 0, 0);
        if (TuningOpModes.DRIVE_CLASS.equals(SparkFunOTOSDrive.class)) {

            SparkFunOTOSDrive.Params params;

            if (hardwareMap.tryGet(AnalogInput.class, "psibot") != null) {
                params = new PsiParams();
            } else if (hardwareMap.tryGet(AnalogInput.class, "roboticabot") != null) {
                params = new RoboticaParams();
            } else if (hardwareMap.tryGet(AnalogInput.class, "omegabot") != null) {
                params = new OmegaParams();
            } else {
                throw new RuntimeException("Unknown bot");
            }

            SparkFunOTOSDrive drive = new SparkFunOTOSDrive(hardwareMap, beginPose, params);


            waitForStart();

            Actions.runBlocking(
                drive.actionBuilder(beginPose)
                        .splineTo(new Vector2d(30, 30), Math.PI / 2)
                        .splineTo(new Vector2d(0, 60), Math.PI)
                        .build());
        } else if (TuningOpModes.DRIVE_CLASS.equals(TankDrive.class)) {
            TankDrive drive = new TankDrive(hardwareMap, beginPose);

            waitForStart();

            Actions.runBlocking(
                    drive.actionBuilder(beginPose)
                            .splineTo(new Vector2d(30, 30), Math.PI / 2)
                            .splineTo(new Vector2d(0, 60), Math.PI)
                            .build());
        } else {
            throw new RuntimeException();
        }
    }
}
