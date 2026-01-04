package org.firstinspires.ftc.teamcode.opmodes;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class ImuTest extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
    private DcMotor smallLaunchMotor;

    private DcMotor bigLaunchMotor;
    private DcMotor intakeMotor;
    private Servo leftTrigger;
    private Servo rightTrigger;

    private IMU imu;

    private VoltageSensor voltageSensor;

    private AprilTagProcessor aprilTag;
    private VisionPortal visionPortal;
    private YawPitchRollAngles cameraOrientation;
    private Position cameraPosition;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private final double smallLaunchSpeed = .7;

    public IMU.Parameters myIMUparameters = new IMU.Parameters(
            new RevHubOrientationOnRobot(
                    new Orientation(
                            AxesReference.INTRINSIC,
                            AxesOrder.ZYX,
                            AngleUnit.DEGREES,
                            180,
                            0,
                            50,
                            0  // acquisitionTime, not used
                    )
            )
    );





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
        imu = hardwareMap.get(IMU.class, "imu");

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
        imu.initialize(myIMUparameters);

        waitForStart();
        while (opModeIsActive()) {
            YawPitchRollAngles robotAngles = imu.getRobotYawPitchRollAngles();
            telemetry.addData("pitch", robotAngles.getPitch());
            telemetry.addData("roll", robotAngles.getRoll());
            telemetry.addData("yaw", robotAngles.getYaw());
            telemetry.update();
        }


    }
}