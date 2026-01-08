package org.firstinspires.ftc.teamcode.opmodes;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class DecodeTestTeleOp extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor smallLaunchMotor;

    private DcMotor bigLaunchMotor;
    private DcMotor intakeMotor;
    private Servo leftTrigger;
    private Servo rightTrigger;

    private VoltageSensor voltageSensor;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private final double smallLaunchSpeed = .7;

//    private FtcDashboard dash;
//    private List<Action> runningActions;
//    Pose2d initialPose;
//    SparkFunOTOSDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {
        //instantiating
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        smallLaunchMotor = hardwareMap.get(DcMotor.class, "smallLaunchMotor");
        bigLaunchMotor = hardwareMap.get(DcMotor.class, "bigLaunchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
        voltageSensor = hardwareMap.get(VoltageSensor.class, "Control Hub");

        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        //setting drive to break when no power
//        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bigLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        //setting launch motors to run with encoders

        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        //reversing necessary motors
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bigLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        smallLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//         dash = FtcDashboard.getInstance();
//        runningActions = new ArrayList<>();
//        initialPose = new Pose2d(1,0,0);
//        drive = NewDrive(hardwareMap, initialPose);
        leftTrigger.setPosition(.47);
        rightTrigger.setPosition(.515);

        cameraPosition = new Position(DistanceUnit.INCH,
                0, 8, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90 + 19, 0, 0);

        waitForStart();


        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()

                // The following default settings are available to un-comment and edit as needed.
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)
                .setCameraPose(cameraPosition, cameraOrientation)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)
                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second (default)
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second (default)
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        //aprilTag.setDecimation(3);

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableLiveView(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();


        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

        // end method initAprilTag()


        while (opModeIsActive()) {


            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
//            List<Action> newActions = new ArrayList<>();
//            for (Action action : runningActions) {
//                action.preview(packet.fieldOverlay());
//                if (action.run(packet)) {
//                    newActions.add(action);
//                }
//            }
//            runningActions = newActions;
//
//
//            dash.sendTelemetryPacket(packet);
            y = -gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            leftFrontMotor.setPower(y + x + rx);
            leftBackMotor.setPower(y - x + rx);
            rightFrontMotor.setPower(y - x - rx);
            rightBackMotor.setPower(y + x - rx);



            if (gamepad1.a) {
                smallLaunchMotor.setPower(.55);
                //big launch motor is 33% larger radius, so it must be reduced
                bigLaunchMotor.setPower(0.75 * .55);
                intakeMotor.setPower(0);

            }
            if (gamepad1.left_trigger > .1) {
                smallLaunchMotor.setPower(.65);
                //big launch motor is 33% larger radius, so it must be reduced
                bigLaunchMotor.setPower(0.75 * .65);
                intakeMotor.setPower(0);
            }
            if (gamepad1.b) {
                bigLaunchMotor.setPower(0);
                smallLaunchMotor.setPower(0);
            }
            if (gamepad1.x) {
                intakeMotor.setPower(1);
                leftTrigger.setPosition(.47);
                rightTrigger.setPosition(.52);
                bigLaunchMotor.setPower(0);
                smallLaunchMotor.setPower(0);
            }
            if (gamepad1.y) {
                intakeMotor.setPower(0);
            }
            if (gamepad1.left_bumper) {
                leftTrigger.setPosition(.52);
                rightTrigger.setPosition(.449);
            }
            if (gamepad1.right_bumper) {
                leftTrigger.setPosition(.47);
                rightTrigger.setPosition(.515);
            }
            if (gamepad1.dpad_right) {
                for (int i = 0; i < 20000; i++) {
                    leftFrontMotor.setPower(-1);
                    rightFrontMotor.setPower(-1);
                    leftBackMotor.setPower(-1);

                    rightBackMotor.setPower(-1);
                }
            }
            if (gamepad1.dpad_up && !visionPortal.getProcessorEnabled(aprilTag)) {
                visionPortal.setProcessorEnabled(aprilTag, true);
            }
            if (gamepad1.dpad_up && visionPortal.getProcessorEnabled(aprilTag)) {
                visionPortal.setProcessorEnabled(aprilTag, false);
            }
            if (aprilTag.getDetections() != null && !aprilTag.getDetections().isEmpty()) {
                ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
                double x = detections.get(0).center.x;
            }

            while ((x < 250 || x > 350) && aprilTag.getDetections() != null && gamepad1.dpad_down && !aprilTag.getDetections().isEmpty()) {
                    x = aprilTag.getDetections().get(0).center.x;
                    if (x > 275) {
                        leftFrontMotor.setPower(.2);
                        leftBackMotor.setPower(.2);
                    }
                    if (x < 325) {
                        rightFrontMotor.setPower(.2);
                        rightBackMotor.setPower(.2);
                    }
                    if (gamepad1.dpad_down) {
                        break;
                    }

            }
            if (aprilTag.getDetections() != null && !aprilTag.getDetections().isEmpty()) {
                ArrayList<AprilTagDetection> detections = aprilTag.getDetections();
                telemetry.addData("CENTER", detections.get(0).center.x);
                telemetry.addData("BEARING", detections.get(0).ftcPose.bearing);

            } else {
                telemetry.addData("CENTER", "NULL");
                telemetry.addData("BEARING", "NULL");
            }
            telemetry.update();

        }



//            if (gamepad2.dpad_down) {
//                if (drive.pose.position.y <= 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .strafeTo(new Vector2d(0, -12))
//                                    .strafeTo(new Vector2d(-36, -36))
//                            )));
//                } else if (drive.pose.position.y > 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .strafeTo(new Vector2d(0, 12))
//                                    .strafeTo(new Vector2d(-36, 36))
//                            )));
//                }
//            }
//            if (gamepad2.dpad_up) {
//                if (drive.pose.position.y <= 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .strafeTo(new Vector2d(0, -12))
//                                    .strafeTo(new Vector2d(60, -60))
//                            )));
//                } else if (drive.pose.position.y > 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .strafeTo(new Vector2d(0, 12))
//                                    .strafeTo(new Vector2d(60, 60))
//                            )));
//                }
//            }
//            if (gamepad2.dpad_left) {
//                if (drive.pose.position.y <= 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .turnTo(Math.PI-Math.atan2(-60-drive.pose.position.y,-60-drive.pose.position.x ))
//                            )));
//                } else if (drive.pose.position.y > 0) {
//                    runningActions.add(new SequentialAction(
//                            new SleepAction(0.5),
//                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
//                                    .turnTo(Math.atan2(60-drive.pose.position.y,-60-drive.pose.position.x ))
//                            )));
//                }
//            }




        }


    }

