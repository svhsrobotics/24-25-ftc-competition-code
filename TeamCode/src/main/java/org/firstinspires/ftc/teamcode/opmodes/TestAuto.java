package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.SparkFunOTOSDrive.NewDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous
public class TestAuto extends LinearOpMode {

    public class Launcher {
        private DcMotorEx launchmotor;

        public Launcher (HardwareMap hardwareMap) {
            launchmotor = hardwareMap.get(DcMotorEx.class, "launch-motor");
            launchmotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            launchmotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }

        private class LaunchSpinUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    launchmotor.setPower(1);
                }
                return false;
            }
        }

        private class LaunchSpinDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    launchmotor.setPower(0);
                }
                return false;
            }
        }

    }

    private class Intake {

        private DcMotorEx intakemotor;

        public Intake (HardwareMap hardwareMap) {
            intakemotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        }

        private class IntakeOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    intakemotor.setPower(1);
                }
                return false;
            }
        }

    }

    private class Conveyor {

        private DcMotorEx conveyormotor;

        public Conveyor (HardwareMap hardwareMap) {
            conveyormotor = hardwareMap.get(DcMotorEx.class, "intake_motor");
        }

        private class ConveyorOn implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    conveyormotor.setPower(1);
                }
                return false;
            }
        }
        private class ConveyorOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                if (!initialized) {
                    conveyormotor.setPower(0);
                }
                return false;
            }
        }


    }




    private VisionPortal visionPortal;
    private AprilTagProcessor aprilTag;
    private Position cameraPosition = new Position(DistanceUnit.INCH,
            0, 0, 0, 0);
    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
            0, -90, 0, 0);


    private void initAprilTag() {
        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
        builder.addProcessor(aprilTag);
        visionPortal = builder.build();


    }

    @Override
    public void runOpMode() throws InterruptedException {



    }
}