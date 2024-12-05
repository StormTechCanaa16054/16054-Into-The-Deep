package org.firstinspires.ftc.teamcode.PID;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Autonomous(name = "PID Autonomous", group = "Linear OpMode")
public class TestePID extends LinearOpMode {

    private DcMotor slideH;
    private static final double TICKS_PER_CM = 42.8;
    private double kP = 0.01;

    @Override
    public void runOpMode() {
    
        slideH = hardwareMap.get(DcMotor.class, "slideH");
        slideH.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideH.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideH.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideH.setDirection(DcMotorSimple.Direction.REVERSE);

        waitForStart();

        if (opModeIsActive()) {
            moveSlideToPosition(12, 0.5);

            telemetry.addData("Final Encoder Position", slideH.getCurrentPosition());
            telemetry.update();
        }
    }


    private void moveSlideToPosition(double targetCm, double maxPower) {
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
