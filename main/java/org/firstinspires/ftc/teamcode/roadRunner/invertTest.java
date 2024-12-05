package org.firstinspires.ftc.teamcode.roadRunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="invert", group="Linear OpMode")
// @Disabled
public class invertTest extends LinearOpMode {

    DcMotor frontLeft, frontRight, backLeft, backRight;

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

        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.y && !yPressed) {
                inverted = !inverted;
                yPressed = true;
                while (gamepad1.y){

                }
            }

            if (!gamepad1.y) {
                yPressed = false;
            }

            double multiplier = inverted ? -1 : 1;

            if (gamepad1.left_stick_x != 0 || gamepad1.left_stick_y != 0) {
                frontLeft.setPower(multiplier * (gamepad1.left_stick_y + gamepad1.left_stick_x));
                backLeft.setPower(multiplier * (gamepad1.left_stick_y + gamepad1.left_stick_x));
                frontRight.setPower(multiplier * (gamepad1.left_stick_y - gamepad1.left_stick_x));
                backRight.setPower(multiplier * (gamepad1.left_stick_y - gamepad1.left_stick_x));
            }

            if (gamepad1.right_trigger > 0) {
                frontRight.setPower(multiplier * gamepad1.right_trigger);
                frontLeft.setPower(multiplier * - gamepad1.right_trigger);
                backRight.setPower(multiplier * - gamepad1.right_trigger);
                backLeft.setPower(multiplier * gamepad1.right_trigger);
            }

            if (gamepad1.left_trigger > 0) {
                frontRight.setPower(multiplier * - gamepad1.left_trigger);
                frontLeft.setPower(multiplier * gamepad1.left_trigger);
                backLeft.setPower(multiplier * - gamepad1.left_trigger);
                backRight.setPower(multiplier * gamepad1.left_trigger);
            }

            telemetry.addData("Inverted Direction", inverted ? "Yes" : "No");
            telemetry.addData("frontLeft", frontLeft.getPower());
            telemetry.addData("backLeft", backLeft.getPower());
            telemetry.addData("frontRight", frontRight.getPower());
            telemetry.addData("backRight", backRight.getPower());
            telemetry.update();
        }
    }
}