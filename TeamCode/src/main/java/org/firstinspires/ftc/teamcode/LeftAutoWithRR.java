package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.teamcode.SparkFunOTOSDrive.NewDrive;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
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

        public Lift (HardwareMap hardwareMap) {
            lift = hardwareMap.get(DcMotorEx.class, "left_lift");
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
                if (pos < 2000.0) {
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
        public Action LiftUp() {
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
                    outtakeClaw.setPosition(1);
                    initialized = true;
                }

                double pos = outtakeClaw.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < 0.99) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public class OuttakeOpen implements Action {
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
    }

    public class IntakeClaw {
        private Servo intakeClaw;

        public IntakeClaw(HardwareMap hardwareMap) {
            intakeClaw = hardwareMap.get(Servo.class, "outtake_claw");
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
    }

    public class OuttakeElbow {
        private Servo outtakeElbow;

        public OuttakeElbow(HardwareMap hardwareMap) {
            outtakeElbow = hardwareMap.get(Servo.class, "outtake_elbow");
        }

        public class OuttakePickUp implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeElbow.setPosition(0);
                    initialized = true;
                }

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .01) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public class OuttakeDropOff implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    outtakeElbow.setPosition(.5);
                    initialized = true;
                }

                double pos = outtakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .49) {
                    return true;
                } else {
                    return false;
                }
            }
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
                    intakeElbow.setPosition(.8);
                    initialized = true;
                }

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos < .8) {
                    return true;
                } else {
                    return false;
                }
            }
        }
        public class IntakePass implements Action {
            private boolean initialized = false;

            @Override
            public boolean run(@NonNull TelemetryPacket packet) {
                if (!initialized) {
                    intakeElbow.setPosition(.27);
                    initialized = true;
                }

                double pos = intakeElbow.getPosition();
                packet.put("outtakeClaw", pos);
                if (pos > .27) {
                    return true;
                } else {
                    return false;
                }
            }
        }
    }


    public class IntakeSlide {
        private DcMotorEx intakeslide;

        public IntakeSlide (HardwareMap hardwareMap) {
            intakeslide = hardwareMap.get(DcMotorEx.class, "intake_slide");
            intakeslide.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            intakeslide.setDirection(DcMotorSimple.Direction.FORWARD);
        }



    }




    @Override
    public void runOpMode() throws InterruptedException {

        Pose2d initialPose = new Pose2d(60, 54, Math.toRadians(90));
        SparkFunOTOSDrive drive = NewDrive(hardwareMap, initialPose);

        OuttakeClaw outclaw = new OuttakeClaw(hardwareMap);

        IntakeClaw inClaw = new IntakeClaw(hardwareMap);







    }
}
