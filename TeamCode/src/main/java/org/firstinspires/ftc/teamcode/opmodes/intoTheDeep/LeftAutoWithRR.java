package org.firstinspires.ftc.teamcode.opmodes.intoTheDeep;

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

@Autonomous
public class LeftAutoWithRR extends LinearOpMode {

    public class Lift {
        private DcMotorEx lift;

        public Lift(HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "left_lift");
            lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            lift.setDirection(DcMotorSimple.Direction.REVERSE);
        }


        public class LiftUp implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 2500.0) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }

        public class LiftUpSome implements Action {
            // checks if the lift motor has been powered on
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    lift.setPower(0.8);
                    initialized = true;
                }

                // checks lift's current position
                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 900) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    lift.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }

        }

        public Action liftUpSome() {
            return new LiftUpSome();
        }

        public Action liftUp() {
            return new LiftUp();
        }

        public class LiftDown implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    lift.setPower(-0.8);
                    initialized = true;
                }

                double pos = lift.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 100.0) {
                    return true;
                } else {
                    lift.setPower(0);
                    return false;
                }
            }
        }

        public Action liftDown() {
            return new LiftDown();
        }
    }


    public class OuttakeClaw {
        private Servo outtakeClaw;

        public OuttakeClaw(HardwareMap hardwareMap) {
            outtakeClaw = hardwareMap.get(Servo.class, "outtake_grab");
        }

        public class OuttakeClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("OuttakeClaw", "Entered run-close");

                if (!initialized) {
                    outtakeClaw.setPosition(0);
                    initialized = true;
                }
//                sleep(1000);

                double pos = outtakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > 0.01) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakeCLose() {
            return new OuttakeClose();
        }

        public class OuttakeOpen implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("OuttakeClaw", "Entered run-open");

                if (!initialized) {
                    outtakeClaw.setPosition(1);
                    initialized = true;
                }
                //sleep(1000);

                double pos = outtakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .99) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakeopen() {
            return new OuttakeOpen();
        }
    }

    public class IntakeClaw {
        private Servo intakeClaw;

        public IntakeClaw(HardwareMap hardwareMap) {
            intakeClaw = hardwareMap.get(Servo.class, "intake_claw");
        }

        public class IntakeClose implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("IntakeClaw", "Entered run-close");

                if (!initialized) {
                    intakeClaw.setPosition(0);
                    initialized = true;
                }
//                sleep(1000);

                double pos = intakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > 0.1) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action intakeClose() {
            return new IntakeClose();
        }

        public class IntakeOpen implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("IntakeClaw", "Entered run-open");
                if (!initialized) {
                    intakeClaw.setPosition(1);
                    initialized = true;
                }
//                sleep(1000);

                double pos = intakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .99) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action intakeOopen() {
            return new IntakeOpen();
        }
    }

    public class OuttakeElbow {
        private Servo outtakeElbow;

        public OuttakeElbow(HardwareMap hardwareMap) {
            outtakeElbow = hardwareMap.get(Servo.class, "deposit_wrist");
        }


        public class OuttakePickUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("OuttakeElbow", "Entered run-PickUp");

                if (!initialized) {
                    outtakeElbow.setPosition(.465);
                    initialized = true;
                }
//                sleep(1000);

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .466) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakePickup() {
            return new OuttakePickUp();
        }

        public class OuttakeHold implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("OuttakeClaw", "Entered run-Hold");

                if (!initialized) {
                    outtakeElbow.setPosition(.7);
                    initialized = true;
                }
//                sleep(1000);

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .7) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakeHold() {
            return new OuttakeHold();
        }

        public class OuttakeDropOff implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("OuttakeElbow", "Entered run-DropOff");

                if (!initialized) {
                    outtakeElbow.setPosition(.7);
                    initialized = true;
                }
//                sleep(1000);


                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .69) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakeDropOff() {
            return new OuttakeDropOff();
        }
    }

    public class IntakeElbow {
        private Servo intakeElbow;

        public IntakeElbow(HardwareMap hardwareMap) {
            intakeElbow = hardwareMap.get(Servo.class, "intake_wrist");
        }

        public class IntakePickUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("IntakeElbow", "Entered run-Pickup");

                if (!initialized) {
                    intakeElbow.setPosition(.95);
                    initialized = true;
                }
//                sleep(1000);

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .94) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action intakePickUp() {
            return new IntakePickUp();
        }

        public class IntakePass implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                android.util.Log.d("IntakeElbow", "Entered run-Pass");
                if (!initialized) {
                    intakeElbow.setPosition(.12);
                    initialized = true;
                }

//                sleep(1000);

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .16) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action intakePass() {
            return new IntakePass();
        }
    }


    public class IntakeSlide {
        private DcMotorEx intakeslide;

        public IntakeSlide(HardwareMap hardwareMap) {
            intakeslide = hardwareMap.get(DcMotorEx.class, "intake_slide");
            intakeslide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            intakeslide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            intakeslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }

        public class IntakeSlideToGrab implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeslide.setPower(0.2);
                    initialized = true;
                }

                // checks lift's current position
                double pos = intakeslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos < 500) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    intakeslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }


        }

        public Action intakeSlideToGrab() {
            return new IntakeSlideToGrab();
        }



        public class IntakeSlideToPass implements Action {
            private boolean initialized = false;

            // actions are formatted via telemetry packets as below
            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                // powers on motor, if it is not on
                if (!initialized) {
                    intakeslide.setPower(-0.2);
                    initialized = true;
                }

                // checks lift's current position
                double pos = intakeslide.getCurrentPosition();
                packet.put("liftPos", pos);
                if (pos > 110) {
                    // true causes the action to rerun
                    return true;
                } else {
                    // false stops action rerun
                    intakeslide.setPower(0);
                    return false;
                }
                // overall, the action powers the lift until it surpasses
                // 3000 encoder ticks, then powers it off
            }




        }
        public Action intakeSlideToPass() {
            return new IntakeSlideToPass();
        }
    }


    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(-36, -60, Math.toRadians(0));
        SparkFunOTOSDrive drive = NewDrive(hardwareMap, initialPose);

        OuttakeClaw outclaw = new OuttakeClaw(hardwareMap);

        IntakeClaw inClaw = new IntakeClaw(hardwareMap);

        IntakeSlide inSlide = new IntakeSlide(hardwareMap);

        Lift lift = new Lift(hardwareMap);

        OuttakeElbow outElbow = new OuttakeElbow(hardwareMap);

        IntakeElbow inElbow = new IntakeElbow(hardwareMap);

        WaitTrajectory w = new WaitTrajectory(hardwareMap, initialPose);


        TrajectoryActionBuilder tab1 = drive.actionBuilder(initialPose)
                .strafeTo(new Vector2d(-36, -54))
                .strafeTo(new Vector2d(-55, -55))
                .turn(Math.toRadians(45));

        TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(new Vector2d(-55, -55), Math.toRadians(45)))
                .turn(Math.toRadians(45))
                .strafeTo(new Vector2d(-40, 0))
                .turn(Math.toRadians(90));
              //  .waitSeconds(2);
//                .waitSeconds(1)
//                .turn(Math.toRadians(-45))
//                .strafeTo(new Vector2d(-54, -54))
//                .waitSeconds(1)
//                .turn(Math.toRadians(45))
//                .strafeTo(new Vector2d(-54, -44))
//                .waitSeconds(1)
//                .strafeTo(new Vector2d(-54, -54))
//                .turn(Math.toRadians(-45));

        TrajectoryActionBuilder tab3 = drive.actionBuilder(new Pose2d(new Vector2d(-55, -55), Math.toRadians(90)))
                .strafeTo(new Vector2d(-52.2, -45));

        TrajectoryActionBuilder waitTwoSecond = drive.actionBuilder(initialPose)
                .waitSeconds(2);
        TrajectoryActionBuilder waitOneSecond = drive.actionBuilder(initialPose)
                .waitSeconds(1);

        TrajectoryActionBuilder waitHalfSecond = drive.actionBuilder(initialPose)
                .waitSeconds(.5);
        TrajectoryActionBuilder drivebacksome = drive.actionBuilder(new Pose2d(new Vector2d(-36, 0), Math.toRadians(180)))
                .strafeTo(new Vector2d(-30, -0));
        TrajectoryActionBuilder tab4 = drive.actionBuilder(new Pose2d(new Vector2d(-49, -45), Math.toRadians(90)))
                .strafeTo(new Vector2d(-55, -55))
                .turn(Math.toRadians(-45));


        Action trajectoryActionCloseout = tab1.endTrajectory().fresh().build();


        waitForStart();


        if (isStopRequested()) return;

//        Action trajectoryActionChosen;
//        trajectoryActionChosen = tab1.build();

        Actions.runBlocking(
                new SequentialAction(
                        inElbow.intakePickUp(),
                        outclaw.outtakeCLose(),
                        outElbow.outtakePickup(),
                        tab1.build(),
                        lift.liftUp(),
                        outElbow.outtakeDropOff(),

                        waitTwoSecond.build(),


                        outclaw.outtakeopen(),
                        outElbow.outtakePickup(),
                        waitHalfSecond.build(),
                        lift.liftDown(),

                        tab3.build(),

                        outclaw.outtakeopen(),
                        //waitOneSecond.build(),
                        w.waitSeconds(1),



                        inElbow.intakePickUp(),
                       // waitOneSecond.build(),
                        //w.waitSeconds(1),

                        inClaw.intakeOopen(),
                    //    waitOneSecond.build(),
                       // w.waitSeconds(1),

                        inSlide.intakeSlideToGrab(),
                    //    waitTwoSecond.build(),
                        w.waitSeconds(1),

                        inClaw.intakeClose(),
                    //    waitTwoSecond.build(),
                        w.waitSeconds(1),

                        inElbow.intakePass(),
                    //    waitTwoSecond.build(),
                        w.waitSeconds(1),
                        inSlide.intakeSlideToPass(),
                        w.waitSeconds(.5),

                        outclaw.outtakeCLose(),
//                        waitTwoSecond.build(),
                        w.waitSeconds(.5),

                        inClaw.intakeOopen(),
//                        waitTwoSecond.build(),
                        w.waitSeconds(.5),

                        inElbow.intakePickUp(),
//                        waitOneSecond.build(),
                        w.waitSeconds(.5),

                        tab4.build(),
                        inElbow.intakePickUp(),
                        lift.liftUp(),
                        outElbow.outtakeDropOff(),

                        w.waitSeconds(2),


                        outclaw.outtakeopen(),
                        outElbow.outtakePickup(),
                        w.waitSeconds(.5),
                        lift.liftDown(),
                        trajectoryActionCloseout)
        );


    }

}


