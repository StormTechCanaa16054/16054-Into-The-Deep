/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode.twoWheelsMr;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates a teleop OpMode for an Omni robot.
 * An external "Robot" class is used to manage all motor/sensor interfaces, and to assist driving functions.
 * The IMU gyro is used to stabilize the heading when the operator is not requesting a turn.
 */

@TeleOp(name="Teleop", group = "Mr. Phil")
public class SampleTeleop extends LinearOpMode {
    double SAFE_DRIVE_SPEED = 1; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    double SAFE_STRAFE_SPEED = 1; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    double SAFE_YAW_SPEED = 0.8; // Adjust this to your robot and your driver.  Slower usually means more accuracy.  Max value = 1.0
    final double HEADING_HOLD_TIME = 10.0; // How long to hold heading once all driver input stops. (This Avoids effects of Gyro Drift)

    // local parameters
    ElapsedTime stopTime = new ElapsedTime();  // Use for timeouts.
    boolean autoHeading = false; // used to indicate when heading should be locked.

    // get an instance of the "Robot" class.
    SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);


    /*DcMotor corredica1, corredica2, intake;
    Servo trava, caixa, corre1, corre2, aviao;
    RevBlinkinLedDriver led;*/

    boolean isPressed = false;
    int invertMove = 1;

    @Override
    public void runOpMode() {
        //led = hardwareMap.get(RevBlinkinLedDriver.class, "led");

      /*  intake = hardwareMap.dcMotor.get("axial");
        corredica1 = hardwareMap.dcMotor.get("corre1");
        corredica2 = hardwareMap.dcMotor.get("corre2");
        corredica1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        corredica2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        trava = hardwareMap.servo.get("trava");
        caixa = hardwareMap.servo.get("caixa");
        corre1 = hardwareMap.servo.get("correE");
        corre2 = hardwareMap.servo.get("correD");
        aviao = hardwareMap.servo.get("aviao");

        corre1.setDirection(Servo.Direction.REVERSE);
        corre2.setDirection(Servo.Direction.FORWARD);
        trava.setDirection(Servo.Direction.REVERSE);
        caixa.setDirection(Servo.Direction.FORWARD);*/


        // Initialize the drive hardware & Turn on telemetry
        robot.initialize(true);

        // Wait for driver to press start
        waitForStart();
        while (opModeInInit()) {
            telemetry.addData(">", "Touch Play to drive");

            // Read and display sensor data
            robot.readSensors();
            telemetry.update();
        }


       /* trava.setPosition(0);
        caixa.setPosition(0.45);
        corre1.setPosition(0.1);
        corre2.setPosition(0.1);
        aviao.setPosition(0.1);*/
        //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);

        while (opModeIsActive()) {
          /*  if (gamepad2.left_stick_y < -0.1) {
                //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.BLUE);
            }
            //if (gamepad2.right_stick_y < 0) {
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.RED);
        }
        if (gamepad2.left_stick_y < 0.1 && gamepad2.left_stick_y > -0.1) {
            corredica1.setPower(-0.1);
            corredica2.setPower(0.1);
        } else {
            corredica1.setPower(gamepad2.left_stick_y);
            corredica2.setPower(-gamepad2.left_stick_y);
        }*/
        /*intake.setPower(gamepad2.right_stick_y);

        if (gamepad2.back) {
            trava.setPosition(0);
            caixa.setPosition(0.45);
            corre1.setPosition(0.1);
            corre2.setPosition(0.1);
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.WHITE);
        }
        if (gamepad2.dpad_down) {
            corre1.setPosition(0.5);
            corre2.setPosition(0.5);
            caixa.setPosition(0.4);
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        if (gamepad2.dpad_left) {
            corre1.setPosition(0.5);
            corre2.setPosition(0.5);
            caixa.setPosition(0.38);
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);
        }
        if (gamepad2.dpad_up) {
            corre1.setPosition(0.5);
            corre2.setPosition(0.5);
            caixa.setPosition(0.32);
            //led.setPattern(RevBlinkinLedDriver.BlinkinPattern.GREEN);

        }
        if (gamepad2.a) {
            trava.setPosition(0.025);
        }
        if (gamepad2.b) {
            trava.setPosition(0.16);
        }*/
            if (gamepad1.right_trigger > 0) {
                SAFE_DRIVE_SPEED = 0.2;
                SAFE_STRAFE_SPEED = 0.2;
                SAFE_YAW_SPEED = 0.2;
            } else {
                SAFE_DRIVE_SPEED = 1;
                SAFE_STRAFE_SPEED = 1;
                SAFE_YAW_SPEED = 0.8;


                robot.readSensors();

                // Allow the driver to reset the gyro by pressing both small gamepad buttons
                if (gamepad1.options && gamepad1.share) {
                    robot.resetHeading();
                    robot.resetOdometry();
                }

                /*Aprender sobre este if */
                if (gamepad1.y && !isPressed) {
                    isPressed = true;
                    if (invertMove == 1) {
                        invertMove = -1;
                    } else {
                        invertMove = 1;
                    }
                }
                if (!gamepad1.y) {
                    isPressed = false;
                }
                double drive = -gamepad1.left_stick_y * SAFE_DRIVE_SPEED * invertMove;      //  Fwd/back on left stick
                double strafe = -gamepad1.left_stick_x * SAFE_STRAFE_SPEED * invertMove;     //  Left/Right on left stick
                double yaw = -gamepad1.right_stick_x * SAFE_YAW_SPEED;       //  Rotate on right stick

                //  OR... For special conditions, Use the DPAD to make pure orthoginal motions
                if (gamepad1.dpad_left) {
                    strafe = SAFE_DRIVE_SPEED / 2.0;
                } else if (gamepad1.dpad_right) {
                    strafe = -SAFE_DRIVE_SPEED / 2.0;
                } else if (gamepad1.dpad_up) {
                    drive = SAFE_DRIVE_SPEED / 2.0;
                } else if (gamepad1.dpad_down) {
                    drive = -SAFE_STRAFE_SPEED / 2.0;
                }

                // This is where we heep the robot heading locked so it doesn't turn while driving or strafing in a straight line.
                // Is the driver turning the robot, or should it hold its heading?

                if (Math.abs(yaw) > 0.05) {
                    // driver is commanding robot to turn, so turn off auto heading.
                    autoHeading = false;
                } else {
                    // If we are not already locked, wait for robot to stop rotating (<2 deg per second) and then lock-in the current heading.
                    if (!autoHeading && Math.abs(robot.getTurnRate()) < 2.0) {
                        robot.yawController.reset(robot.getHeading());  // Lock in the current heading
                        autoHeading = true;
                    }
                }

                // If auto heading is on, override manual yaw with the value generated by the heading controller.
                // if (autoHeading) {
                //yaw = robot.yawController.getOutput(robot.getHeading());
                // }

                //  Drive the wheels based on the desired axis motions
                robot.moveRobot(drive, strafe, yaw);

                // If the robot has just been sitting here for a while, make heading setpoint track any gyro drift to prevent rotating.
                if ((drive == 0) && (strafe == 0) && (yaw == 0)) {
                    if (stopTime.time() > HEADING_HOLD_TIME) {
                        robot.yawController.reset(robot.getHeading());  // just keep tracking the current heading
                    }
                } else {
                    stopTime.reset();
                }
            }
        }
    }
}



