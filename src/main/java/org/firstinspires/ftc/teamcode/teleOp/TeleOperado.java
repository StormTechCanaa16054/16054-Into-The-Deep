package org.firstinspires.ftc.teamcode.teleOp;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="TeleOp", group="Linear OpMode")
// @Disabled
public class TeleOperado extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

    DcMotor slideVL, slideVR;
    DcMotor slideH;
    Servo servo0, servo1, servo2, servo3;

    boolean servoActive = false;
    boolean buttonPressed = false;
    boolean inverted = false;
    boolean yPressed = false;

    @Override
    public void runOpMode() {

        frontLeft = hardwareMap.get(DcMotor.class, "lf");
        frontLeft.setDirection(DcMotorSimple.Direction.FORWARD);

        frontRight = hardwareMap.get(DcMotor.class, "rf");
        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);

        backRight = hardwareMap.get(DcMotor.class, "rb");
        backRight.setDirection(DcMotorSimple.Direction.REVERSE);

        backLeft = hardwareMap.get(DcMotor.class, "lb");
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);


        servo0 = hardwareMap.get(Servo.class, "gate");

        servo1 = hardwareMap.get(Servo.class, "claw");
        servo1.setDirection(Servo.Direction.REVERSE);

        servo2 = hardwareMap.get(Servo.class, "gyro");
        servo3 = hardwareMap.get(Servo.class, "outTake");

        slideVL = hardwareMap.get(DcMotor.class, "slideVL");
        slideVL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideVR = hardwareMap.get(DcMotor.class, "slideVR");
        slideVR.setDirection(DcMotorSimple.Direction.FORWARD);
        slideVR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideVR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        slideH = hardwareMap.get(DcMotor.class, "slideH");

        waitForStart();

        double Switch = 1.0;
        initWait4Start();

        while (opModeIsActive()) {

            if (gamepad1.y && !yPressed) {
                inverted = !inverted;
                yPressed = true;
            }

            if (!gamepad1.y) {
                yPressed = false;
            }

            double multiplier = inverted ? -1 : 1;

            if (gamepad2.right_trigger > 0) {
                slideH.setPower(-gamepad2.right_trigger);
            } else if (gamepad2.left_trigger > 0) {
                slideH.setPower(gamepad2.left_trigger);
            } else {
                slideH.setPower(0);
            }

            if (gamepad1.right_trigger > 0) {
                frontRight.setPower(gamepad1.right_trigger);
                frontLeft.setPower(-gamepad1.right_trigger);
                backRight.setPower(-gamepad1.right_trigger);
                backLeft.setPower(gamepad1.right_trigger);

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
                frontLeft.setPower(0.8);
                frontRight.setPower(0.8);
                backLeft.setPower(-0.8);
                backRight.setPower(-0.8);
            } else {
                frontLeft.setPower(-0.8);
                backLeft.setPower(-0.8);
                frontRight.setPower(0.8);
                backRight.setPower(0.8);
            }

            if ((gamepad1.right_stick_x > -0.75) || (gamepad1.right_stick_x < 0.25)) {
                if (gamepad1.right_stick_x != 0 || gamepad1.right_stick_y != 0) {
                    frontLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 3);
                    backLeft.setPower((gamepad1.right_stick_y + gamepad1.right_stick_x) / 3);
                    frontRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 3);
                    backRight.setPower((gamepad1.right_stick_y - gamepad1.right_stick_x) / 3);
                }
            } else if (gamepad1.right_stick_x > 0.25) {
                frontLeft.setPower(0.8);
                frontRight.setPower(0.8);
                backLeft.setPower(-0.8);
                backRight.setPower(-0.8);
            } else {
                frontLeft.setPower(-0.8);
                backLeft.setPower(-0.8);
                frontRight.setPower(0.8);
                backRight.setPower(0.8);
            }

            if (gamepad2.x && !buttonPressed) {
                servoActive = !servoActive;
                if (servoActive) {
                    closedClaw();
                } else {
                    openClaw();
                }
                buttonPressed = true;
            }

            if (!gamepad2.x) {
                buttonPressed = false;
            }

            if (gamepad2.back) {
                zeroPos();
            }
            if (gamepad2.y) {
               readyBasket();
            }

           intakeGyro();

            switch (slidePosition) {
                case SecondBasket:
                    SetSlidePosition(3300);
                    if (getSlidePosition() >= 3250 && getSlidePosition() <= 3300) {
                        servo3.setPosition(0.7);
                    }
                    break;

                case Initial:
                    SetSlidePosition(0);
                    break;

                case FirstBasket:
                    SetSlidePosition(1550);
                    if (getSlidePosition() >= 1500 && getSlidePosition() <= 1550) {
                        servo3.setPosition(0.7);
                    }

                    break;
                case Clipper:
                    SetSlidePosition(2100);
            }

            if (gamepad2.dpad_down) {
                initialServo();
                servo0.setPosition(0);
                slidePosition = SlidePosition.Initial;
            } else if (gamepad2.dpad_left) {
                //scoreBasket();
                slidePosition = SlidePosition.FirstBasket;
            } else if (gamepad2.dpad_up) {
                //scoreBasket();
                slidePosition = SlidePosition.SecondBasket;
                initialServo();
            } else if (gamepad2.dpad_right) {
                slidePosition = SlidePosition.Clipper;
            }

            telemetry.addData("frontLeft", frontLeft.getPower());
            telemetry.addData("backLeft", backLeft.getPower());
            telemetry.addData("frontRight", frontRight.getPower());
            telemetry.addData("backRight", backRight.getPower());
            telemetry.update();
        }
    }

    private SlidePosition slidePosition = SlidePosition.Initial;

    private enum SlidePosition {
        Initial,
        SecondBasket,
        FirstBasket,
        Clipper
    }

    private double kP = 0.01;

    private void SetSlidePosition(double newPosition) {
        double ServoPosition = 0.7;
        double position = slideVL.getCurrentPosition();
        double error = getSlidePosition() - newPosition;
        double output = error * kP;

        telemetry.addData("Arm Motor Position", slideVL.getCurrentPosition());
        telemetry.addData("PID Output", output);
        output = MathUtils.clamp(output, -1, 1);
        slideVL.setPower(-output);
        slideVL.setPower(output);
    }

    private void initialServo() {

        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }

    private void initWait4Start() {
        servo1.setPosition(0.25);
        servo0.setPosition(0.77);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }

    private void climberJoy() {
        slideVL.setPower(gamepad2.left_stick_y);
        slideVL.setPower(gamepad2.left_stick_y);
    }

    private void scoreBasket() {

        if (getSlidePosition() >= 1530 && getSlidePosition() <= 1550) {
            servo3.setPosition(0.7);
        }
        if (getSlidePosition() >= 3280 && getSlidePosition() <= 3300) {
            servo3.setPosition(0.7);
        }
    }

    private double getSlidePosition() {
        return slideVL.getCurrentPosition();
    }

    private void closedClaw() {
        servo1.setPosition(0.488);
    }
    private void openClaw() {
        servo1.setPosition(0);
    }
    private void zeroPos() {
        servo0.setPosition(0);
        servo1.setPosition(0);
        servo2.setPosition(0);
        servo3.setPosition(0);
    }
    private void readyBasket() {
        servo0.setPosition(0.73);
        servo2.setPosition(0);
    }
    private void intakeGyro(){
        if (gamepad2.b) {
            servo2.setPosition(0.3);
        } else servo2.setPosition(0);
    }
}