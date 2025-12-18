package org.firstinspires.ftc.teamcode.opmodes.intoTheDeep;



import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp
@Config
public class TestWheels extends LinearOpMode {

    private DcMotor leftFrontMotor;
    private DcMotor rightFrontMotor;
    private DcMotor leftBackMotor;
    private DcMotor rightBackMotor;
//    private DcMotor smallLaunchMotor;

//    private DcMotor bigLaunchMotor;
//    private DcMotor intakeMotor;
//    private Servo leftTrigger;
//    private Servo rightTrigger;


    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
//    private final double smallLaunchSpeed = .05;
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
//        smallLaunchMotor = hardwareMap.get(DcMotor.class, "smallLaunchMotor");
//        bigLaunchMotor = hardwareMap.get(DcMotor.class, "bigLaunchMotor");
//        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
//        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
//        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        //setting drive to break when no power
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting launch motors to run with encoders
//        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reversing necessary motors
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        bigLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);
//        smallLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);

//         dash = FtcDashboard.getInstance();
//        runningActions = new ArrayList<>();
//        initialPose = new Pose2d(1,0,0);
//        drive = NewDrive(hardwareMap, initialPose);
//        leftTrigger.setPosition(.5);
//        rightTrigger.setPosition(.49);
        waitForStart();





        while(opModeIsActive() ) {
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
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            rightBackMotor.setPower((y + x + rx));
            rightFrontMotor.setPower((y - x + rx));
            leftBackMotor.setPower((y - x - rx));
            leftFrontMotor.setPower((y + x - rx));

            while (gamepad1.a) {
               leftFrontMotor.setPower(1);
            }
            while (gamepad1.b) {
                rightFrontMotor.setPower(1);

            }
            while (gamepad1.x) {
                leftBackMotor.setPower(1);
            }
            while (gamepad1.y) {
                rightBackMotor.setPower(1);
            }
//            if (gamepad1.left_bumper) {
//                leftTrigger.setPosition(.52);
//                rightTrigger.setPosition(.47);
//            }
//            if (gamepad1.right_bumper) {
//                leftTrigger.setPosition(.47);
//                rightTrigger.setPosition(.52);
//            }
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

}