package org.firstinspires.ftc.teamcode;

import static java.lang.String.format;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

@TeleOp
public class BobTest extends LinearOpMode {

    AprilTagProcessor tagProcessor;
    VisionPortal visionPortal;
    List<AprilTagDetection> witnessedTags = new ArrayList<>();
    Telemetry tagInfo;
    ElapsedTime time = new ElapsedTime();

    public void init(HardwareMap hwMap, Telemetry tagInfo) {
        this.tagInfo = tagInfo;
        tagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class, "Webcam 1"));
        builder.setCameraResolution(new Size(640, 480));
        builder.addProcessor(tagProcessor);

        visionPortal = builder.build();
    }

    @Override
    public void runOpMode() throws InterruptedException {

        init(hardwareMap, telemetry);

        waitForStart();

        while (!isStopRequested()) {
            telemetry.update();

            witnessedTags = tagProcessor.getDetections();

            for (AprilTagDetection detection : witnessedTags) {
                if (detection.metadata != null) {
                    telemetry.addData("ID", detection.metadata.name);
                    telemetry.addData("X", detection.ftcPose.x);
                    telemetry.addData("Y", detection.ftcPose.y);
                    telemetry.addData("Z", detection.ftcPose.z);
                    telemetry.addData("Pitch", detection.ftcPose.pitch);
                    telemetry.addData("Roll", detection.ftcPose.roll);
                    telemetry.addData("Yaw", detection.ftcPose.yaw);
                } else {
                    telemetry.addData("Unknown ", detection.id);
                }
            }
        }
    }
}

/*
 public class BobTest extends OpMode {

    DcMotor leftFront;
    DcMotor leftBack;
    DcMotor rightFront;
    DcMotor rightBack;
    double y;
    double x;
    double rx;
    double dPad;

    @Override
    public void init() {
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftFront.setDirection(DcMotor.Direction.FORWARD);
        rightBack.setDirection(DcMotor.Direction.FORWARD);
        leftBack.setDirection(DcMotor.Direction.FORWARD);
        rightFront.setDirection(DcMotor.Direction.FORWARD);
    }

    public void loop() {

        rx = -gamepad1.right_stick_x;
        y = -gamepad1.left_stick_x;
        x = gamepad1.left_stick_y;

        if (gamepad1.dpad_up) {
            dPad = 1;
        }
        if (gamepad1.dpad_right) {
            dPad = 2;
        }
        if (gamepad1.dpad_down) {
            dPad = 3;
        }
        if (gamepad1.dpad_left) {
            dPad = 4;
        }
        telemetry.addData("dPad = ", dPad);

        if (dPad == 1) {
            //good
            leftFront.setPower(0.75 * (y + x + rx));
        }
        if (dPad == 2) {
            //good
            leftBack.setPower(0.75 * (y - x + rx));
        }
        if (dPad == 3) {
            //good
            rightFront.setPower(0.75 * (y - x - rx));
        }
        if (dPad == 4) {
            //good
            rightBack.setPower(0.75 * (y + x - rx));
        }
    }
 }
 */
