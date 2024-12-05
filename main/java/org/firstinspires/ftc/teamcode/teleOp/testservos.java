package org.firstinspires.ftc.teamcode.teleOp;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name = "servos", group = "Linear OpMode")
public class testservos extends LinearOpMode {
    Servo servo0, servo1, servo2,servo3;
    DcMotor slideH;
    DcMotor slideVL, slideVR;

    DcMotor frontRight;
    DcMotor frontLeft;
    DcMotor backRight;
    DcMotor backLeft;

    public void FowardMode(double speed) {
        frontRight.setPower(speed);
        frontLeft.setPower(speed);
        backRight.setPower(speed);
        backLeft.setPower(speed);
    }

    public void StrafeMode(double speed) {
        frontRight.setPower(speed);
        frontLeft.setPower(-speed);
        backRight.setPower(-speed);
        backLeft.setPower(speed);
    }

    boolean isServoActive = false;
    boolean buttonPreviouslyPressed = false;

    @Override
    public void runOpMode() {
        slideVL = hardwareMap.get(DcMotor.class, "slideVL");
        slideVR = hardwareMap.get(DcMotor.class, "slideVR");

        servo0 = hardwareMap.get(Servo.class, "portal");
        servo1 = hardwareMap.get(Servo.class, "claw");
        servo2 = hardwareMap.get(Servo.class, "gyro");
        servo3 = hardwareMap.get(Servo.class, "outTake");

        frontLeft = hardwareMap.get(DcMotorEx.class, "lf");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        frontRight = hardwareMap.get(DcMotorEx.class, "rf");
        frontRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backRight = hardwareMap.get(DcMotorEx.class, "rb");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft = hardwareMap.get(DcMotorEx.class, "lb");
        backLeft.setDirection(DcMotorSimple.Direction.FORWARD);


        slideVL = hardwareMap.get(DcMotor.class, "slideVL");
        slideVR = hardwareMap.get(DcMotor.class, "slideVR");

        slideH = hardwareMap.get(DcMotor.class, "slideH");

        slideVL.setDirection(DcMotorSimple.Direction.REVERSE);
        waitForStart();
        servo1.setPosition(0.25);
        servo0.setPosition(0.75);
        servo2.setPosition(0);
        servo3.setPosition(0);

        while (opModeIsActive()) {

            while (opModeIsActive()) {



                if (gamepad1.a && !buttonPreviouslyPressed) {
                    isServoActive = !isServoActive;
                    if (isServoActive) {
                        servo1.setPosition(1.0);
                    } else {
                        servo1.setPosition(0.0);
                    }
                    buttonPreviouslyPressed = true;
                }

                if (!gamepad1.a) {
                    buttonPreviouslyPressed = false;
                }
            }

            slideVL.setPower(gamepad2.left_stick_y);
            slideVR.setPower(gamepad2.left_stick_y);

            slideH.setPower(gamepad2.right_stick_x);

            if(gamepad1.right_trigger > 0){
                StrafeMode(gamepad1.right_trigger);
            }

            if(gamepad1.left_trigger > 0){
                StrafeMode(-gamepad1.left_trigger);
            }

            if(gamepad1.left_stick_y > 0.1){
                FowardMode(gamepad1.left_stick_y);
            }

            if(-gamepad1.left_stick_y > 0.1) {
                FowardMode(gamepad1.left_stick_y);
            }

            if (gamepad1.right_trigger > 0) {
                frontRight.setPower(gamepad1.right_trigger);
                frontLeft.setPower(-gamepad1.right_trigger);
                backRight.setPower(gamepad1.right_trigger);
                backLeft.setPower(-gamepad1.right_trigger);

            }

            if (gamepad1.left_trigger > 0) {
                frontRight.setPower(-gamepad1.left_trigger);
                frontLeft.setPower(gamepad1.left_trigger);
                backLeft.setPower(-gamepad1.left_trigger);
                backRight.setPower(gamepad1.left_trigger);

            }

            if ((gamepad1.left_stick_x > -0.75) || (gamepad1.left_stick_x < 0.25)) {
                if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                    frontLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                    backLeft.setPower(gamepad1.left_stick_y + gamepad1.left_stick_x);
                    frontRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                    backRight.setPower(gamepad1.left_stick_y - gamepad1.left_stick_x);
                } else if ((gamepad1.right_trigger == 0) && (gamepad1.left_trigger == 0) &&
                        (gamepad1.right_stick_x == 0) && (gamepad1.right_stick_y == 0)) {
                    frontLeft.setPower(0);
                    backLeft.setPower(0);
                    frontRight.setPower(0);
                    backRight.setPower(0);
                }
            } else if (gamepad1.left_stick_x > 0) {
                frontLeft.setPower(0.7);
                frontRight.setPower(0.7);
                backLeft.setPower(-0.7);
                backRight.setPower(-0.7);
            } else {
                frontLeft.setPower(-0.7);
                backLeft.setPower(-0.7);
                frontRight.setPower(0.7);
                backRight.setPower(0.7);
            }

            if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
                if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                    frontLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 2);
                    frontRight.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 2);
                    backLeft.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 2);
                    backRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 2);
                }
            } else if (gamepad1.right_stick_x > 0) {
                frontLeft.setPower(0.7);
                backLeft.setPower(0.7);
                frontRight.setPower(-0.7);
                backRight.setPower(-0.7);

            } else {
                frontLeft.setPower(-0.7);
                backLeft.setPower(-0.7);
                frontRight.setPower(0.7);
                backRight.setPower(0.7);

            }

            if (gamepad2.back) {
                servo0.setPosition(0);
                servo1.setPosition(0);
                servo2.setPosition(0);
                servo3.setPosition(0);
            }
            if (gamepad2.b) {
                servo1.setPosition(0.25);
            }
            if (gamepad2.dpad_up) {
                servo0.setPosition(0.75);
                servo2.setPosition(0);
            }
            if (gamepad2.dpad_left) {
                servo2.setPosition(0.285);
            }
            if (gamepad2.left_bumper) {
                servo3.setPosition(0.7);
            }
        }
    }
}
