package org.firstinspires.ftc.teamcode.opmodes.IntoTheDeep;


import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.util.Toggle;

@TeleOp
public class TrumansSkibidiDriveModes extends LinearOpMode {
    private DcMotor topLeftMotor;
    private DcMotor topRightMotor;
    private DcMotor bottomLeftMotor;
    private DcMotor bottomRightMotor;
    private CRServo tail;
    @Override
    public void runOpMode() throws InterruptedException {
        Toggle myToggleButton = new Toggle();
        double ly;
        double lx;
        double rx;
        double ry;
        boolean leftBumper;
        boolean rightBumper; //why? just use if statements       /////why not.....
        topLeftMotor = hardwareMap.get(DcMotor.class,"FL");
        topRightMotor = hardwareMap.get(DcMotor.class,"FR");
        bottomLeftMotor = hardwareMap.get(DcMotor.class,"BL");
        bottomRightMotor = hardwareMap.get(DcMotor.class,"BR");
        topLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        bottomLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //tail = hardwareMap.get(CRServo.class,"servo"); ONLY FOR DOG
        //double tailAngle = 1;
        double motorSpeed = 0.3; //^^
        waitForStart();

        while (opModeIsActive()){

            myToggleButton.update(gamepad1.a);

            if (myToggleButton.state){
                lx=gamepad1.right_stick_y;
                ly=gamepad1.right_stick_y;
                rx=gamepad1.left_stick_y;
                ry=gamepad1.left_stick_y;
                leftBumper=gamepad1.left_bumper;
                rightBumper=gamepad1.right_bumper;
                if (leftBumper) {
                    ly=-1;
                    lx=1;
                }
                if (rightBumper) {
                    rx=1;
                    ry=-1;

                }
                topLeftMotor.setPower(ly);
                bottomLeftMotor.setPower(lx);
                topRightMotor.setPower(rx);
                bottomRightMotor.setPower(ry);
                telemetry.addData("Ly", ly);
                telemetry.addData("Lx", lx);
                telemetry.addData("Rx", rx);
                telemetry.addData("Ry", ry);
                telemetry.update();
            }
            else{
                double y = -gamepad1.left_stick_y; // Y stick is reversed!
                double x = gamepad1.left_stick_x;
                double rx2 = gamepad1.right_stick_x;

                topLeftMotor.setPower(y+ x + rx2);
                bottomLeftMotor.setPower(y - x + rx2);
                topRightMotor.setPower(y - x - rx2);
                bottomRightMotor.setPower(y + x - rx2);
                // remember kids plagiarism is wrong
            }

            // while(!check){
                // put the other stuff that isn't t-bar steering here
            }
        }
}




