package org.firstinspires.ftc.teamcode.robotica2025modular;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * RoboticaTeleOp: Robotica's DecodeTestTeleOp (2025-26, DECODE), reorganized into parts.
 *
 * SAME BEHAVIOR as opmodes/DecodeTestTeleOp.java (robotica branch, last commit Feb 12 2026):
 * same buttons, same numbers, same order of steps each pass, same Driver Station text. The
 * difference is where the code lives:
 *   - THIS file decides WHAT to do from the controller ("X was pressed: start intaking").
 *   - The part files know HOW ("start intaking" means intake on, triggers back, flywheels off).
 *       Drivetrain  the four wheels          Launcher  two flywheels + two trigger servos
 *       Intake      the intake motor         Aimer     camera, AprilTag, how to turn
 *       RoboticaRobot  builds all four, and holds the moves that use several parts
 * Things marked "NOTE (same as Robotica)" are kept exactly as Robotica had them, on purpose
 * (Mark, 2026-10-26): the split shows the same code, oddities included, so the team can find
 * them. ONE FIX: centering on the tag no longer freezes the TeleOp (see RoboticaRobot).
 *
 * Left out because they never did anything, or aren't allowed in matches:
 *   - FTC Dashboard (lines 5 to 6, 26, 177): a packet built every pass and never sent. This
 *     season, rule R704.D bans Dashboard during matches; telemetry goes through the Driver
 *     Station only.
 *   - Road Runner plans, all commented out (lines 52 to 55, 95 to 98, 182 to 192, 305 to 353).
 *   - The battery voltage sensor (line 69, found but never read).
 *   - Lines 260 to 263: a tag check that makes a new x inside the "if" and never uses it.
 *
 * HOW A LinearOpMode RUNS: the SDK calls runOpMode() once when INIT is pressed. Our code runs
 * top to bottom: set up, wait for START, then loop until STOP. Every pass of the loop runs
 * quickly and never waits, so the driver never loses control.
 *
 * CONTROLLER (gamepad1):
 *   Left stick ............. drive forward/back and strafe     Right stick left/right .. turn
 *   A ...................... spin up (adjustable speed), intake off
 *   Left trigger ........... spin up (fixed 0.65), intake off
 *   B ...................... flywheels off       X ......... intake on, triggers back, flywheels off
 *   Hold D-pad left + B / X  adjustable speed up / down (instead of B / X's usual jobs)
 *   Y ...................... intake off
 *   Left bumper / right .... triggers release / rest
 *   D-pad right ............ back up at full power while held
 *   D-pad up ............... tag reader off (see Aimer.toggleCamera)
 *   Hold D-pad down ........ turn to center the AprilTag
 */
@TeleOp
public class RoboticaTeleOp extends LinearOpMode {

    private RoboticaRobot robot = new RoboticaRobot();

    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);   // finds and sets up every device; the triggers move (see there)

        waitForStart();

        // NOTE (same as Robotica, lines 107 to 165): the camera starts AFTER START, so it's
        // still warming up when the match begins. The fix is to move this line into
        // RoboticaRobot.init(), so the camera starts during INIT.
        robot.aimer.init(hardwareMap);

        while (opModeIsActive()) {

            // ---- 1. Drive from the sticks (lines 193 to 200) -----------------------------
            // Pushing a stick UP gives a NEGATIVE number (a game-controller convention),
            // so the minus sign makes "stick up" = positive = forward.
            double forward = -gamepad1.left_stick_y;
            double strafe = gamepad1.left_stick_x;
            double turn = gamepad1.right_stick_x;
            robot.drive.drive(forward, strafe, turn);

            // ---- 2. Launcher and intake buttons (lines 204 to 236) -----------------------
            // While D-pad left is held, B and X fine-tune the speed instead of their usual jobs.
            if (gamepad1.a && !gamepad1.dpad_left) {
                robot.getReadyToShoot(robot.launcher.getTargetSpeed());
            }
            if (gamepad1.left_trigger > .1) {   // a trigger is analog (0 to 1); 0.1 ignores a light touch
                robot.getReadyToShoot(0.65);
            }
            if (gamepad1.b && !gamepad1.dpad_left) {
                robot.launcher.turnOff();
            }
            if (gamepad1.x && !gamepad1.dpad_left) {
                robot.startIntaking();
            }
            if (gamepad1.b && gamepad1.dpad_left) {
                robot.launcher.changeTargetSpeed(.00001);
            }
            if (gamepad1.x && gamepad1.dpad_left) {
                robot.launcher.changeTargetSpeed(-.00001);
            }
            if (gamepad1.y) {
                robot.intake.stop();
            }

            // ---- 3. Triggers (lines 237 to 244) ------------------------------------------
            if (gamepad1.left_bumper) {
                robot.launcher.release();
            }
            if (gamepad1.right_bumper) {
                robot.launcher.rest();
            }

            // ---- 4. D-pad right: back up (lines 245 to 253) ------------------------------
            if (gamepad1.dpad_right) {
                robot.backUp();
            }

            // ---- 5. D-pad up: camera on/off (lines 254 to 259) ---------------------------
            // NOTE (same as Robotica): it always ends OFF. See Aimer.toggleCamera().
            if (gamepad1.dpad_up) {
                robot.aimer.toggleCamera();
            }

            // ---- 6. Read the camera (lines 265 to 270) -----------------------------------
            robot.update();

            // ---- 7. Hold D-pad down: center on the tag (lines 272 to 287) ----------------
            // FIXED (Mark, 2026-10-26): one small turn per pass instead of a loop that froze
            // everything. See RoboticaRobot.centerOnTag(). Same rule as the old loop: only
            // while D-pad down is held and the tag isn't centered yet.
            if (gamepad1.dpad_down && !robot.aimer.isCentered()) {
                robot.centerOnTag();
            }

            // ---- 8. Driver Station (lines 289 to 299) ------------------------------------
            // addData only queues a line; telemetry.update() sends them all.
            if (robot.aimer.canSeeTarget()) {
                telemetry.addData("CENTER", robot.aimer.getTagCenterX());
            } else {
                telemetry.addData("CENTER", "NULL");
            }
            // NOTE (same as Robotica): labeled "POWER", but it's the speed passed to setVelocity
            // (see Launcher.turnOn).
            telemetry.addData("POWER", robot.launcher.getTargetSpeed());
            telemetry.update();
        }
    }
}
