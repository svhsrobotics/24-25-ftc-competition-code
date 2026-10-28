package org.firstinspires.ftc.teamcode.gamma2025modular;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/*
 * BobTeleOp: Haisen's BobComp TeleOp (2025-26, DECODE), reorganized into parts.
 *
 * SAME BEHAVIOR as opmodes/BobComp.java (final version, Feb 12 2026): same buttons, same
 * numbers, same order of steps each pass, same Driver Station text. The difference is
 * where the code lives:
 *   - THIS file decides WHAT to do from the controller ("A was pressed: launcher on").
 *   - The part files know HOW ("on" means setVelocity on two motors).
 *       Drivetrain  the four wheels        Launcher  flywheels + pushers
 *       Intake      the intake motor       Aimer     camera, AprilTag, how to turn
 *       BobRobot    builds all four with one call
 * Things marked "NOTE (same as Bob)" are kept exactly as Bob had them, on purpose.
 * Left out because they never did anything: OGTargeting() (never called), the unused
 * "tagInfo" field, the battery sensor (read but never used), setTargetPosition(0).
 *
 * HOW AN OpMode RUNS (the SDK calls these, our code never calls them itself):
 *   INIT pressed  -> init() once
 *   START pressed -> start() once
 *   then          -> loop() over and over, about 50 times a second, until STOP
 * Every pass of loop() runs top to bottom quickly and never waits, so the driver
 * never loses control.
 *
 * CONTROLLER (gamepad1):
 *   Left stick up/down ...... drive forward/back      Left stick left/right ... turn
 *   Right stick left/right .. strafe                  Triggers ................ intake (right one way, left the other)
 *   Bumpers ................. speed preset 750 / 900  D-pad up/down/right/left  speed +10 / -10 / +50 / -50
 *   A / B ................... launcher on / off       Hold X .................. push
 *   Hold Y .................. auto-aim (and auto speed when locked on)
 *   Click both sticks ....... swap target goal (blue/red)
 */
@TeleOp
public class BobTeleOp extends OpMode {

    private BobRobot robot = new BobRobot();

    // Button memory. These must be fields (not variables inside loop()) because they have to
    // remember what happened on the PREVIOUS pass. Variables inside loop() vanish every pass.
    private boolean dPadPressed = false;       // was any D-pad direction held last pass?
    private boolean targetSwapping = false;    // were both sticks still clicked from a swap?

    @Override
    public void init() {
        robot.init(hardwareMap);   // finds and sets up every device. Nothing moves yet.
    }

    @Override
    public void start() {
        dPadPressed = false;
        targetSwapping = false;
    }

    @Override
    public void loop() {

        // ---- 1. Launcher status on the Driver Station screen ----------------------------
        // addData only queues a line; nothing shows until telemetry.update() at the end.
        // NOTE (same as Bob): labeled "Power", but it's the target speed in ticks per second.
        telemetry.addData("Shooting Power", robot.launcher.getTargetSpeed());
        if (robot.launcher.isAtSpeed()) {
            telemetry.addLine("Launcher Powered!");
        } else {
            telemetry.addLine("Launcher is not powered!");
        }

        // ---- 2. Read the driving sticks -------------------------------------------------
        // Pushing a stick UP gives a NEGATIVE number (a game-controller convention),
        // so the minus sign makes "stick up" = positive = forward.
        // (BobComp called these y, rx and x. On Bob, the left stick drives and turns,
        // and the right stick strafes.)
        double forward = -gamepad1.left_stick_y;
        double turn = gamepad1.left_stick_x;
        double strafe = gamepad1.right_stick_x;

        // ---- 3. Launcher speed: D-pad nudges, once per press ----------------------------
        // Without dPadPressed, holding the button for half a second (about 25 passes) would
        // add 250 instead of 10. The flag lets one press count once, then waits until every
        // D-pad direction is let go. ADDING needs this; SETTING (the bumpers below) doesn't.
        // (SDK 12 can do this for you: gamepad1.dpadUpWasPressed(). See FIRST's sample
        // ConceptGamepadEdgeDetection.java.)
        if (gamepad1.dpad_up && !dPadPressed) {
            dPadPressed = true;
            robot.launcher.changeTargetSpeed(10);
        }
        if (gamepad1.dpad_down && !dPadPressed) {
            dPadPressed = true;
            robot.launcher.changeTargetSpeed(-10);
        }
        if (gamepad1.dpad_right && !dPadPressed) {
            dPadPressed = true;
            robot.launcher.changeTargetSpeed(50);
        }
        if (gamepad1.dpad_left && !dPadPressed) {
            dPadPressed = true;
            robot.launcher.changeTargetSpeed(-50);
        }
        if (!(gamepad1.dpad_up || gamepad1.dpad_down || gamepad1.dpad_right || gamepad1.dpad_left)) {
            dPadPressed = false;
        }

        // Bumper presets. Holding a bumper sets the same number every pass, which is harmless.
        if (gamepad1.left_bumper) {
            robot.launcher.setTargetSpeed(Launcher.PRESET_LOW);
        }
        if (gamepad1.right_bumper) {
            robot.launcher.setTargetSpeed(Launcher.PRESET_HIGH);
        }

        // ---- 4. Pushers: hold X to push, let go to rest ---------------------------------
        if (!gamepad1.x) {
            robot.launcher.rest();
        } else {
            robot.launcher.push();
        }

        // ---- 5. Launcher on (A) / off (B) -----------------------------------------------
        // Two buttons and one remembered on/off, like a light with separate On and Off
        // switches. No one-press logic needed. (If both are held, B wins because it's second.)
        if (gamepad1.a) {
            robot.launcher.turnOn();
        }
        if (gamepad1.b) {
            robot.launcher.turnOff();
        }
        robot.launcher.update();   // send this pass's target speed (or 0) to the wheels

        // ---- 6. Intake on the triggers --------------------------------------------------
        // Right trigger one way, left trigger the other; how far it's pulled = how fast.
        robot.intake.run((gamepad1.right_trigger * -1) + (gamepad1.left_trigger * 1));

        // ---- 7. Swap target goal: click BOTH sticks -------------------------------------
        // A tiny two-state machine. "Waiting": watch for both sticks clicked, then swap.
        // "Just swapped": do nothing until both are let go. Both sticks at once is nearly
        // impossible to do by accident mid-match.
        if (!targetSwapping) {
            if (gamepad1.left_stick_button
                    && gamepad1.right_stick_button) {
                targetSwapping = true;
                robot.aimer.swapTarget();
            }
        } else {
            if (!gamepad1.left_stick_button
                    && !gamepad1.right_stick_button) {
                targetSwapping = false;
            }
        }

        // ---- 8. Show the target: text, and the controller's light color -----------------
        // setLedColor(red, green, blue, milliseconds). It runs every pass, so it stays lit.
        // Only works on controllers with a light bar (PS4/PS5 style).
        if (robot.aimer.isTargetingBlue()) {
            telemetry.addLine("Target: BLUE");
            gamepad1.setLedColor(0, 0, 255, 300);
        } else {
            telemetry.addLine("Target: RED");
            gamepad1.setLedColor(255, 0, 0, 300);
        }

        // ---- 9. Look for the goal's AprilTag --------------------------------------------
        robot.aimer.update();
        if (robot.aimer.canSeeTarget()) {
            telemetry.addData("Target Heading: ", robot.aimer.getBearing());
            telemetry.addData("Target Distance: ", robot.aimer.getDistance());
        } else {
            telemetry.addLine("Target Heading: N/A");
            telemetry.addLine("Target Distance: N/A");
        }

        // ---- 10. Each pass: auto-aim OR drive -------------------------------------------
        // Decided fresh every pass. If the tag disappears while Y is held, the very next
        // pass goes back to normal driving, so the driver never gets stuck.
        if (gamepad1.y && robot.aimer.canSeeTarget()) {
            robot.drive.turnInPlace(robot.aimer.turnPower());   // sticks are ignored meanwhile
        } else {
            robot.drive.drive(forward, strafe, turn);
        }

        // ---- 11. Locked on? Buzz, and pick the speed from the distance ------------------
        // rumble(15) = buzz for 15 ms, every pass while locked: a steady buzz the driver
        // can feel without looking at the screen.
        if (robot.aimer.isLockedOn()) {
            telemetry.addLine("Target Locked");
            gamepad1.rumble(15);

            // NOTE (same as Bob): step 5 already sent this pass's speed to the wheels, so a
            // speed picked here reaches them on the NEXT pass, about 20 ms later.
            if (gamepad1.y) {
                robot.launcher.setSpeedForDistance(robot.aimer.getDistance());
            }
        } else {
            // NOTE (same as Bob): this also shows when the tag IS seen but not lined up.
            // "Not locked" would describe it better.
            telemetry.addLine("Target Out of Sight");
        }

        // ---- 12. Send this pass's lines to the Driver Station ---------------------------
        // Sends everything added above, then clears the list so the next pass starts fresh.
        telemetry.update();
    }
}
