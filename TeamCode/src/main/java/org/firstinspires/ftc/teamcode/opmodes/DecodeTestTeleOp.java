package org.firstinspires.ftc.teamcode.opmodes;



import static org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.SparkFunOTOSDrive.NewDrive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.InstantAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.hardware.sparkfun.SparkFunOTOS;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.MecanumDrive;
import org.firstinspires.ftc.teamcode.opmodes.intoTheDeep.SparkFunOTOSDrive;

import java.util.ArrayList;
import java.util.List;

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


    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;
    private FtcDashboard dash = FtcDashboard.getInstance();
    private List<Action> runningActions = new ArrayList<>();
    Pose2d initialPose = new Pose2d(1,0,0);
    SparkFunOTOSDrive drive = NewDrive(hardwareMap, initialPose);



    @Override
    public void runOpMode() throws InterruptedException {
        //instantiating
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        smallLaunchMotor = hardwareMap.get(DcMotor.class, "small_launch_motor");
        bigLaunchMotor = hardwareMap.get(DcMotor.class, "big_launch_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        rightTrigger = hardwareMap.get(Servo.class, "rightTrigger");
        leftTrigger = hardwareMap.get(Servo.class, "leftTrigger");

        //setting drive to break when no power
        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //setting launch motors to run with encoders
        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //reversing necessary motors
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bigLaunchMotor.setDirection(DcMotorSimple.Direction.REVERSE);


        waitForStart();

        while(opModeIsActive() ) {
            TelemetryPacket packet = new TelemetryPacket();

            // updated based on gamepads

            // update running actions
            List<Action> newActions = new ArrayList<>();
            for (Action action : runningActions) {
                action.preview(packet.fieldOverlay());
                if (action.run(packet)) {
                    newActions.add(action);
                }
            }
            runningActions = newActions;


            dash.sendTelemetryPacket(packet);
            y = gamepad1.left_stick_y;
            x = gamepad1.left_stick_x;
            rx = gamepad1.right_stick_x;

            rightBackMotor.setPower((y + x + rx));
            rightFrontMotor.setPower((y - x + rx));
            leftBackMotor.setPower((y - x - rx));
            leftFrontMotor.setPower((y + x - rx));

            if (gamepad1.a) {
                smallLaunchMotor.setPower(1);
                bigLaunchMotor.setPower(0.75);
            }
            if (gamepad1.b) {
                bigLaunchMotor.setPower(0);
                smallLaunchMotor.setPower(0);
            }
            if (gamepad1.x) {
                intakeMotor.setPower(1);
            }
            if (gamepad1.y) {
                intakeMotor.setPower(0);
            }
            if (gamepad1.left_bumper) {
                leftTrigger.setPosition(.1);
                rightTrigger.setPosition(-.1);
            }
            if (gamepad1.right_bumper) {
                leftTrigger.setPosition(-.1);
                rightTrigger.setPosition(.1);
            }
            if (gamepad2.dpad_down) {
                if (drive.pose.position.y <= 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .strafeTo(new Vector2d(0, -12))
                                    .strafeTo(new Vector2d(-36, -36))
                            )));
                } else if (drive.pose.position.y > 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .strafeTo(new Vector2d(0, 12))
                                    .strafeTo(new Vector2d(-36, 36))
                            )));
                }
            }
            if (gamepad2.dpad_up) {
                if (drive.pose.position.y <= 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .strafeTo(new Vector2d(0, -12))
                                    .strafeTo(new Vector2d(60, -60))
                            )));
                } else if (drive.pose.position.y > 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .strafeTo(new Vector2d(0, 12))
                                    .strafeTo(new Vector2d(60, 60))
                            )));
                }
            }
            if (gamepad2.dpad_left) {
                if (drive.pose.position.y <= 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .turnTo(Math.PI-Math.atan2(-60-drive.pose.position.y,-60-drive.pose.position.x ))
                            )));
                } else if (drive.pose.position.y > 0) {
                    runningActions.add(new SequentialAction(
                            new SleepAction(0.5),
                            new InstantAction(() -> drive.actionBuilder(new Pose2d(drive.pose.position.x, drive.pose.position.y, 0))
                                    .turnTo(Math.atan2(60-drive.pose.position.y,-60-drive.pose.position.x ))
                            )));
                }
            }


        }

    }

}