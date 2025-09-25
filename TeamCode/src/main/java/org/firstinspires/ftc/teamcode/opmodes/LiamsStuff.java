

        package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.util.Debouncer;
import org.firstinspires.ftc.teamcode.util.Toggle;

        @TeleOp
public class LiamsStuff extends LinearOpMode {

    private DcMotor right;
    private DcMotor left;
    private DcMotor launch1;
    private DcMotor launch2;
    private final Debouncer craigUp = new Debouncer();
    private final Debouncer craigDown = new Debouncer();
    private Toggle stan = new Toggle();

    /**
     * This OpMode offers Tank Drive style TeleOp control for a direct drive robot.
     *
     * In this Tank Drive mode, the left and right joysticks (up
     * and down) drive the left and right motors, respectively.
     */
    @Override
    public void runOpMode() {
        double launcher_speed = 0.8;

        right = hardwareMap.get(DcMotor.class, "right");
        left = hardwareMap.get(DcMotor.class, "left");
        launch1 = hardwareMap.get(DcMotor.class, "launch1");
        launch2 = hardwareMap.get(DcMotor.class, "launch2");
//todo: push to robot
        // Reverse one of the drive motors.
        // You will have to determine which motor to reverse for your robot.
        // In this example, the right motor was reversed so that positive
        // applied power makes it move the robot in the forward direction.
        right.setDirection(DcMotor.Direction.REVERSE);
        waitForStart();

            // Put run blocks here.


            while (opModeIsActive()) {
                boolean toggle = false;
                if (gamepad1.a){
                    toggle = true;
                }
                else if (gamepad1.b){
                    toggle = false;
                }
                // Put loop blocks here.
                // The Y axis of a joystick ranges from -1 in its topmost position to +1 in its bottommost position.
                // We negate this value so that the topmost position corresponds to maximum forward power.
                left.setPower(-gamepad1.left_stick_y);
                right.setPower(-gamepad1.right_stick_y);

                if(toggle){
                    launch2.setPower(launcher_speed);
                    launch1.setPower(launcher_speed * -1);}


                if (!toggle) {
                    left.setPower(0);
                    right.setPower(0);
                }
                ;
                ;


                    if (craigDown.update(gamepad1.dpad_up)) {
                        launcher_speed = launcher_speed + .05;
                        if (launcher_speed > 1) {
                            launcher_speed = 1;
                        }

                    } else if (craigUp.update(gamepad1.dpad_down)) {
                        launcher_speed = launcher_speed - .05;
                        if (launcher_speed < 0.3) {
                            launcher_speed = 0.3;
                        }
                    }

                telemetry.addData("Left Pow", left.getPower());
                telemetry.addData("Right Pow", right.getPower());
                telemetry.addData("launcher speed", launcher_speed);
                telemetry.addData("craigup", craigUp.lastState);
                telemetry.addData("craigdown", craigDown.lastState);
                telemetry.update();
            }
        }
    }


