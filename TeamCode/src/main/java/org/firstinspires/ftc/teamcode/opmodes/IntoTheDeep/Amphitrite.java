package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.util.PIDController2;
import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class Amphitrite extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        final DcMotor outSlide = hardwareMap.get(DcMotor.class, "out");
        final DcMotor upSlide = hardwareMap.get(DcMotor.class, "notheight");
        final Servo outServo = hardwareMap.get(Servo.class, "outClawServo");
        final Servo outClawRotationServo = hardwareMap.get(Servo.class, "outClawRotationServo");
        final Servo upViperSlideArm = hardwareMap.get(Servo.class, "arm");
        final Servo upClaw = hardwareMap.get(Servo.class, "upclaw");
        final DcMotor leftFrontMotor = hardwareMap.get(DcMotor.class, "leftFront");
        final DcMotor rightFrontMotor = hardwareMap.get(DcMotor.class, "rightFront");
        final DcMotor leftBackMotor = hardwareMap.get(DcMotor.class, "leftBack");
        final DcMotor rightBackMotor = hardwareMap.get(DcMotor.class, "rightBack");
        final PIDController2 PidV = new PIDController2(0,0.01,0,1);
        //TODO: TUNE THE FREAKING LOOP FOR THE "OUT" SLIDE
        final PIDController2 PidH = new PIDController2(0, 0.01, 0, 1);
        final Toggle slidetToggle = new Toggle();
        double outReference = 0;
        double upReference = 0;




        upViperSlideArm.setDirection(Servo.Direction.REVERSE);

        outSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outSlide.setDirection(DcMotorSimple.Direction.REVERSE);

        outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        outSlide.setTargetPosition(0);

        //
        upSlide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        upSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        upSlide.setDirection(DcMotorSimple.Direction.REVERSE);
        //non reversed wheels
        rightFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFrontMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        leftBackMotor.setDirection(DcMotorSimple.Direction.REVERSE);



        waitForStart();
       outClawRotationServo.setPosition(0.5);
       upClaw.setPosition(1);

        while (opModeIsActive()) {
            slidetToggle.update(gamepad1.left_stick_button);
            //out slide
            if (gamepad1.left_trigger != 0 ) {
                // outReference = outReference - 10 * (double) gamepad1.left_trigger;
                outSlide.setTargetPosition(outSlide.getCurrentPosition() - 1000 * (int) gamepad1.left_trigger);
            } else if (gamepad1.right_trigger != 0) {
                //outReference = outReference + 10* (double) gamepad1.right_trigger;
                outSlide.setTargetPosition(outSlide.getCurrentPosition() + 1000 * (int) gamepad1.right_trigger);
            }

            if (outSlide.getTargetPosition() >= 3000) {
                outSlide.setTargetPosition(3000);
            } else if (outSlide.getTargetPosition() <= 0) {
                outSlide.setTargetPosition(0);
            }
        /*if (outReference >= 4890){
            outReference = 4890;}
        if(outReference <= 0){
            outReference = 0;}*/
            if (upReference <= 0) {
                upReference = 0;
            }
            outSlide.setPower(1);
            //outSlide.setPower(PidV.usePIDLoop(outSlide.getCurrentPosition(), outReference));
            //up slide
            if (gamepad2.left_trigger !=0 ) {
                upReference = upReference - 10 * (double) gamepad2.left_trigger;
            } else if (gamepad2.right_trigger != 0 ) {
                upReference = upReference + 10 * (double) gamepad2.right_trigger;
            }
            if (upReference >= 9000) {
                upReference = 9000;
            }
            upSlide.setPower(PidV.usePIDLoop(upSlide.getCurrentPosition(), upReference));
            //out claw stuff


            if (gamepad1.left_bumper ) {
                outServo.setPosition(0.3);
            } else if (gamepad1.right_bumper ) {
                outServo.setPosition(0);
            }
            if (gamepad1.dpad_left) {
                outClawRotationServo.setPosition(outClawRotationServo.getPosition() - 0.01);
            } else if (gamepad1.dpad_right) {
                outClawRotationServo.setPosition(outClawRotationServo.getPosition() + 0.01);
            }
            if(gamepad1.dpad_up){
                outClawRotationServo.setPosition(0.5);//todo make sure this is the right pos even though it probably isn't
            }
            if(gamepad1.dpad_down) {outClawRotationServo.setPosition(0.9);}
            //outClawRotationServo.setPosition(outClawRotationServo.getPosition());
            //up servos


            if (gamepad2.left_bumper) {
                upClaw.setPosition(0.7);
            } else if (gamepad2.right_bumper) {
                upClaw.setPosition(1);
            }
            if (gamepad2.dpad_left) {
                upViperSlideArm.setPosition(upViperSlideArm.getPosition() - 0.01);
            } else if (gamepad2.dpad_right) {
                upViperSlideArm.setPosition(upViperSlideArm.getPosition() + 0.01);
            }

            if(gamepad2.dpad_up){
                upViperSlideArm.setPosition(0.4); ;//Todo check the number, and make sure this doesn't need to be a loop
            }

            if(gamepad2.dpad_down){
                upViperSlideArm.setPosition(0.1);  //todo ^^
            }
            //passthrough! :P

            if (gamepad1.a) {
                // outSlide is not in the correct position (range) OR upSlide is not at (exactly) 0
                while ((outSlide.getCurrentPosition() < 1070 || outSlide.getTargetPosition() > 1090) ||
                        upSlide.getCurrentPosition() > 10 ) {
                    outSlide.setTargetPosition(1070);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outSlide.setPower(1);
                    upReference = 0;
                    upSlide.setPower(PidV.usePIDLoop(upSlide.getCurrentPosition(), upReference));

                    //android.util.Log.d("OpModeDbg", "Inside Loop A");

                    //outServo.setPosition(0.5);
                }
               // android.util.Log.d("OpModeDbg", "Finished Loop A!");
                upClaw.setPosition(0.7);



                outClawRotationServo.setPosition(0.3);
                sleep(1000);
                upViperSlideArm.setPosition(0.12);
                sleep(1000);
                upClaw.setPosition(1);
                sleep(500);
                outServo.setPosition(0.5);
            }

            //gp1B macro
            if(gamepad1.b){
                while(outSlide.getCurrentPosition() <2990 || outSlide.getCurrentPosition() > 3010){
                outClawRotationServo.setPosition(0.5);//todo: change this when it is inevitably wrong
                outSlide.setTargetPosition(3000);
                outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                outSlide.setPower(1);}
            }
            //gp1X
            if(gamepad1.x){
                while(outSlide.getCurrentPosition() >= 10){
                    outSlide.setTargetPosition(0);
                    outSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outSlide.setPower(1);
                    outClawRotationServo.setPosition(0.5); //todo again need to check this value
                }
            }
            //gp2B macro
            if(gamepad2.b){
                while(upSlide.getCurrentPosition() < 8300){

                upReference = 8500;
                upViperSlideArm.setPosition(0.3);
                upSlide.setPower(PidV.usePIDLoop(upSlide.getCurrentPosition(), upReference));} //todo: check this number
            }
            //gp2X
            if(gamepad2.x){
                while(upSlide.getCurrentPosition() > 10){
                    upReference = 0;
                    upViperSlideArm.setPosition(0.7);
                    upSlide.setPower(PidV.usePIDLoop(upSlide.getCurrentPosition(), upReference));
                    // todo check this
                }
            }


            double y1 = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x1 = gamepad1.left_stick_x;
            double rx1 = gamepad1.right_stick_x;
            double y2 = -gamepad2.left_stick_y; // Remember, Y stick is reversed!
            double x2 = gamepad2.left_stick_x;
            double rx2 = gamepad2.right_stick_x;
            telemetry.addData("y1", y1);
            telemetry.addData("x1", x1);
            telemetry.addData("rx1", rx1);
            telemetry.addData("y2", y2);
            telemetry.addData("x2", x2);
            telemetry.addData("rx2", rx2);

            if(gamepad1.left_stick_x !=0 || gamepad1.left_stick_y !=0 || gamepad1.right_stick_x !=0 || gamepad1.right_stick_y !=0){

                leftFrontMotor.setPower(y1 + x1 + rx1);
                leftBackMotor.setPower(y1 - x1 + rx1);
                rightFrontMotor.setPower(y1 - x1 - rx1);
                rightBackMotor.setPower(y1 + x1 - rx1);
            }
            else  {
                leftFrontMotor.setPower(y2 + x2 + rx2);
                leftBackMotor.setPower(y2 - x2 + rx2);
                rightFrontMotor.setPower(y2 - x2 - rx2);
                rightBackMotor.setPower(y2 + x2 - rx2);
            }
            //telemetry :P
            telemetry.addData("out reference", outReference);
            telemetry.addData("up reference", upReference);
            telemetry.addData("left trigger", gamepad1.left_trigger);
            telemetry.addData("right trigger", gamepad1.right_trigger);
            telemetry.addData("out pos", outSlide.getCurrentPosition());
            telemetry.addData("up Pos", upSlide.getCurrentPosition());
            telemetry.addData("arm pos", upViperSlideArm.getPosition());
            telemetry.addData("out target pos", outSlide.getTargetPosition());
            telemetry.addData("up target pos", upReference);
            telemetry.addData("out pos", outSlide.getCurrentPosition());
            telemetry.addData("up pos", upSlide.getCurrentPosition());
            telemetry.addData("wrist pos", outClawRotationServo.getPosition());

            telemetry.update();


        }

    }

}