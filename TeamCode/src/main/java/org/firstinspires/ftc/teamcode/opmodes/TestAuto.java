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
import org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.SparkFunOTOSDrive;
import org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.TankDrive;
import org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.WaitTrajectory;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

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
        public Action launchSpinUp() {
            return new LaunchSpinUp();
        }
        public Action launchSpinDown() {
            return new LaunchSpinDown();
        }

    }

    private class Intake {

        private DcMotorEx intakemotor;

        public Intake (HardwareMap hardwareMap) {
            intakemotor = hardwareMap.get(DcMotorEx.class, "intake-motor");
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
        public Action intakeOn() {
            return new IntakeOn();
        }

    }

    private class Conveyor {

        private DcMotorEx conveyormotor;

        public Conveyor (HardwareMap hardwareMap) {
            conveyormotor = hardwareMap.get(DcMotorEx.class, "conveyor-motor");
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
        public Action conveyorOn() {
            return new ConveyorOn();
        }
        public Action conveyorOff() {
            return new ConveyorOff();
        }


    }




//    private VisionPortal visionPortal;
//    private AprilTagProcessor aprilTag;
//    private Position cameraPosition = new Position(DistanceUnit.INCH,
//            0, 0, 0, 0);
//    private YawPitchRollAngles cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
//            0, -90, 0, 0);


//    private void initAprilTag() {
//        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//
//
//    }

//    private void telemetryAprilTag() {
//
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());
//
//        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                        detection.robotPose.getPosition().x,
//                        detection.robotPose.getPosition().y,
//                        detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop
//
//        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//
//    }   // end method telemetryAprilTag()

 // end class
    @Override
    public void runOpMode() throws InterruptedException {
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Conveyor conveyor = new Conveyor(hardwareMap);
//        aprilTag = new AprilTagProcessor.Builder().setCameraPose(cameraPosition, cameraOrientation).build();
//        VisionPortal.Builder builder = new VisionPortal.Builder();
//        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));
//        builder.addProcessor(aprilTag);
//        visionPortal = builder.build();
//        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
//        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
//        for (AprilTagDetection detection : currentDetections) {
//            if (detection.metadata != null) {
//                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
//                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)",
//                        detection.robotPose.getPosition().x,
//                        detection.robotPose.getPosition().y,
//                        detection.robotPose.getPosition().z));
//                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)",
//                        detection.robotPose.getOrientation().getPitch(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getRoll(AngleUnit.DEGREES),
//                        detection.robotPose.getOrientation().getYaw(AngleUnit.DEGREES)));
//            } else {
//                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
//                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
//            }
//        }   // end for() loop

        // Add "key" information to telemetry
//        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
//        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
//
//        telemetryAprilTag();

//        Pose2d initialPose = new Pose2d(currentDetections.get(0).robotPose.getPosition().x, currentDetections.get(0).robotPose.getPosition().y,currentDetections.get(0).robotPose.getOrientation().getYaw());
        Pose2d initialPose = new Pose2d(1,0,0);
        telemetry.addLine(String.format("Pose", initialPose.position.x));
        SparkFunOTOSDrive drive = NewDrive(hardwareMap, initialPose);
        WaitTrajectory w = new WaitTrajectory(hardwareMap, initialPose);

        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-12, 60)); //blue small launch zone
        TrajectoryActionBuilder tab2 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(12, 60)); //red small launch zone
        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(-12, 60, 0))
                .strafeTo(new Vector2d(-60, 60)); //blue human player from launch zone
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(12, 60, 0))
                .strafeTo(new Vector2d(60, 60)); //red human player from launch zone
        TrajectoryActionBuilder wait = drive.actionBuilder(initialPose)
                .waitSeconds(3);

        Action trajectoryActionCloseout = tab1.endTrajectory().fresh().build();

        waitForStart();

        if (isStopRequested()) {
            return;
        }
        if (initialPose.position.x < 0) {
            Actions.runBlocking(
                    new SequentialAction(
                            launcher.launchSpinUp(),
                            tab1.build(),
                            conveyor.conveyorOn(),
                            wait.build(),
                            tab3.build()

                    )
            );
        } else if (initialPose.position.x > 0) {
            Actions.runBlocking(
                    new SequentialAction(
                            launcher.launchSpinUp(),
                            tab2.build(),
                            conveyor.conveyorOn(),
                            wait.build(),
                            tab4.build()

                    )
            );
        }

    }


    }
