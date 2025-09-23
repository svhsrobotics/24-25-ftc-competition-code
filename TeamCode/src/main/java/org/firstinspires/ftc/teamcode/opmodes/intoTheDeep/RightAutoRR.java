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
public class RightAutoRR extends LinearOpMode {



    private Servo sweep1;

    private Servo sweep2;


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
                telemetry.addData("LIFTUP: ", pos);
                telemetry.update();
                packet.put("liftPos", pos);
                if (pos < 1300) {
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
                if (!initialized) {
                    outtakeClaw.setPosition(0);
                    initialized = true;
                }

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
                if (!initialized) {
                    outtakeClaw.setPosition(1);
                    initialized = true;
                }

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
                if (!initialized) {
                    intakeClaw.setPosition(1);
                    initialized = true;
                }

                double pos = intakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < 0.99) {
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
                if (!initialized) {
                    intakeClaw.setPosition(0);
                    initialized = true;
                }

                double pos = intakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > 0.01) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action intakeOpen() {
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
                if (!initialized) {
                    outtakeElbow.setPosition(.45);
                    initialized = true;
                }

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .46) {
                    return true;
                } else {
                    return false;
                }
            }
        }

        public Action outtakePickup() {
            return new OuttakePickUp();
        }

        public class OuttakeDropOff implements Action {

            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeElbow.setPosition(.7);
                    initialized = true;
                }

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

        public class OuttakeSpecimanDrop implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeElbow.setPosition(.6);
                    initialized = true;
                }

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .59) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public Action outtakeSpecimanDrop() {
            return new OuttakeSpecimanDrop();
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
                if (!initialized) {
                    intakeElbow.setPosition(.95);
                    initialized = true;
                }

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (intakeElbow.getPosition() < .94) {
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
                if (!initialized) {
                    intakeElbow.setPosition(.13);
                    initialized = true;
                }

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (intakeElbow.getPosition() > .14) {
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
                if (pos > 200) {
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

            public Action intakeSlideToPass() {
                return new IntakeSlideToPass();
            }


        }
    }
        @Override
        public void runOpMode() throws InterruptedException {

            sweep1 = hardwareMap.get(Servo.class, "sweep1");
            sweep2 = hardwareMap.get(Servo.class, "sweep2");

            sweep1.setPosition(.38);
            sweep2.setPosition(.51);

            Pose2d initialPose = new Pose2d(12, -60, Math.toRadians(0));
            SparkFunOTOSDrive drive = NewDrive(hardwareMap, initialPose);

            OuttakeClaw outclaw = new OuttakeClaw(hardwareMap);

            IntakeClaw inClaw = new IntakeClaw(hardwareMap);

            IntakeSlide inSlide = new IntakeSlide(hardwareMap);

            Lift lift = new Lift(hardwareMap);

           OuttakeElbow outElbow = new OuttakeElbow(hardwareMap);

           IntakeElbow inElbow = new IntakeElbow(hardwareMap);


           WaitTrajectory w = new WaitTrajectory(hardwareMap, initialPose);


            TrajectoryActionBuilder tab1 = drive.actionBuilder(new Pose2d(new Vector2d(12, -60),0))
                    .strafeTo(new Vector2d(0, -36))
                    .turn(Math.toRadians(-90))
                    .strafeTo(new Vector2d(0, -28))
                    .waitSeconds(2);

            TrajectoryActionBuilder tab2 = drive.actionBuilder(new Pose2d(new Vector2d(12, -60),0))
                    .strafeTo(new Vector2d(54, -54));





            Action trajectoryActionCloseout = tab1.endTrajectory().fresh().build();


            waitForStart();


            if (isStopRequested()) return;

//        Action trajectoryActionChosen;
//        trajectoryActionChosen = tab1.build();

            Actions.runBlocking(
                    new SequentialAction(
                            outclaw.outtakeCLose(),
                            inElbow.intakePickUp(),
                            tab1.build(),
                            outElbow.outtakeSpecimanDrop(),
                            w.waitSeconds(1),
                            lift.liftUp(),
                            outclaw.outtakeopen(),
                            //lift.liftDown(),
                            outElbow.outtakePickup(),
                            w.waitSeconds(3),
                            tab2.build(),
                            trajectoryActionCloseout
                    )
            );




        }

    }













