package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@Autonomous
@Config
public class DecodeBasicRedFarAuto extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor smallLaunchMotor;

    private DcMotor bigLaunchMotor;
    private DcMotor intakeMotor;
    private Servo leftTrigger;
    private Servo rightTrigger;
    int g;
    int z;
    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        smallLaunchMotor = hardwareMap.get(DcMotor.class, "smallLaunchMotor");
        bigLaunchMotor = hardwareMap.get(DcMotor.class, "bigLaunchMotor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        bigLaunchMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        z = 0;
        g = 0;
        cameraPosition = new Position(DistanceUnit.INCH,
                0, 8, 0, 0);
        cameraOrientation = new YawPitchRollAngles(AngleUnit.DEGREES,
                0, -90 + 19, 0, 0);


        //setting launch motors to run with encoders
        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
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
        waitForStart();
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
        sleep(18000);
            for (int i = 0; i < 11000; i++) {
                leftFrontMotor.setPower(.2);
                rightFrontMotor.setPower(.2);
                leftBackMotor.setPower(.2);
                rightBackMotor.setPower(.2);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);
            for (int i = 0; i < 9000; i++) {
                rightFrontMotor.setPower(-.3);
                rightBackMotor.setPower(-.3);
                leftBackMotor.setPower(.3);
                leftFrontMotor.setPower(.3);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            TelemetryPacket packet = new TelemetryPacket();
            double x = 500;
            ArrayList<AprilTagDetection> april;

            april = aprilTag.getDetections();
            if (!april.isEmpty()) {
                x = april.get(0).center.x;
            }

            while (x == 0 || x >= 500) {
                april = aprilTag.getDetections();
                if (!april.isEmpty()) {
                    x = april.get(0).center.x;
                }
            }
            telemetry.addData("STARTING", x);
            telemetry.update();
int y = 0;
        while ((x < 370 || x > 390)) {
                telemetry.addData("CENTER", x);
                telemetry.addData("Y", y);
                telemetry.update();
                april = aprilTag.getDetections();
                if (!april.isEmpty()) {
                    x = april.get(0).center.x;
                } else {
                    y++;
                }
                if (x > 360) {
                    leftFrontMotor.setPower(.15);
                    leftBackMotor.setPower(.15);
                }
                if (x < 340) {
                    rightFrontMotor.setPower(.15);
                    rightBackMotor.setPower(.15);
                }
                if (y >8000) {
                    break;
                }
            }
        telemetry.addData("CENTER", x);
telemetry.update();

        leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);

            smallLaunchMotor.setPower(.44);
            //big launch motor is 33% larger radius, so it must be reduced
            bigLaunchMotor.setPower(0.75 * .44);
            intakeMotor.setPower(0);

            sleep(6000);
            leftTrigger.setPosition(.52);
            rightTrigger.setPosition(.455);
            sleep(1000);
            for (int i = 0; i < 7000; i++) {
                leftFrontMotor.setPower(1);
                rightFrontMotor.setPower(1);
                leftBackMotor.setPower(1);
                rightBackMotor.setPower(1);
            }
            leftFrontMotor.setPower(0);
            rightFrontMotor.setPower(0);
            leftBackMotor.setPower(0);
            rightBackMotor.setPower(0);





    }
}

