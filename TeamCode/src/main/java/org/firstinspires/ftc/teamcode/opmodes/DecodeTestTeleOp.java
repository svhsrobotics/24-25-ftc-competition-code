package org.firstinspires.ftc.teamcode.opmodes;



import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
    private DcMotor conveyorMotor;

    private double y; // Remember, Y stick is reversed!
    private double x;
    private double rx;


    @Override
    public void runOpMode() throws InterruptedException {
        leftFrontMotor = hardwareMap.get(DcMotor.class, "left_front");
        rightFrontMotor = hardwareMap.get(DcMotor.class, "right_front");
        leftBackMotor = hardwareMap.get(DcMotor.class, "left_back");
        rightBackMotor = hardwareMap.get(DcMotor.class, "right_back");
        smallLaunchMotor = hardwareMap.get(DcMotor.class, "small_launch_motor");
        bigLaunchMotor = hardwareMap.get(DcMotor.class, "big_launch_motor");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        conveyorMotor = hardwareMap.get(DcMotor.class, "conveyor_motor");

        leftFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightFrontMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        leftBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightBackMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        smallLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        bigLaunchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        while(opModeIsActive() ) {
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
                conveyorMotor.setPower(1);
            }
            if (gamepad1.right_bumper) {
                conveyorMotor.setPower(0);
            }

        }

    }

}