/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.autonomousMode;

import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PID.PIDController;
import org.firstinspires.ftc.teamcode.twoWheelsMr.SimplifiedOdometryRobot;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Sample Autonomous", group = "Mr. Phil")
public class blueAutoL extends LinearOpMode {
    private SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    DcMotor frontLeft, frontRight, backLeft, backRight;

    DcMotor slideH;

    Servo servo0, servo1, servo2, servo3;

    private static final double TICKS_PER_CM = 42.8;
    private double kP = 0.01;

    DcMotor slideVL, slideVR;

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

    private void intakeGyro() {
        if (gamepad2.b) {
            servo2.setPosition(0.3);
        } else servo2.setPosition(0);

    }



    @Override public void runOpMode() {

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

        robot.initialize(true);

       /*boolean slidesMoving = true;
        boolean driving = true ;*/

        waitForStart();
        robot.resetHeading();

        if (opModeIsActive()) {
            robot.drive(10, 0, 0, 0);
            moveSlideToPosition(12, 0.5);

            telemetry.addData("Final Encoder Position", slideH.getCurrentPosition());
            telemetry.update();

        }

        }

    private void moveSlideToPosition ( double targetCm, double maxPower){
        double targetTicks = targetCm * TICKS_PER_CM;

        while (opModeIsActive()) {
            double currentPosition = slideH.getCurrentPosition();

            double error = targetTicks - currentPosition;

            double output = kP * error;

            output = Math.max(-maxPower, Math.min(maxPower, output));

            slideH.setPower(output);

            telemetry.addData("Target (ticks)", targetTicks);
            telemetry.addData("Current Position (ticks)", currentPosition);
            telemetry.addData("Error (ticks)", error);
            telemetry.addData("Output (power)", output);
            telemetry.update();

            if (Math.abs(error) < 10) {
                break;
            }
        }
        slideH.setPower(0);
    }
}

