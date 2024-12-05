package org.firstinspires.ftc.teamcode.teleOp;

import androidx.core.math.MathUtils;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp(name="testPID", group="Linear OpMode")
// @Disabled
public class sample extends LinearOpMode{

    DcMotor slideVL;

    @Override
    public void runOpMode() {

        slideVL = hardwareMap.get(DcMotor.class, "slideVL");
        slideVL.setDirection(DcMotorSimple.Direction.FORWARD);
        slideVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        slideVL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        waitForStart();

        double Switch = 1.0;
        while (opModeIsActive()) {

            switch (slidePosition) {
                case SecondBasket:
                    SetSlidePosition(3300);

                    break;

                case Initial:
                    SetSlidePosition(0);
                    break;

                case FirstBasket:
                    SetSlidePosition(1550);

                    break;
                case Clipper:
                    SetSlidePosition(2100);
            }

            if (gamepad2.dpad_down) {
                slidePosition = SlidePosition.Initial;
            } else if(gamepad2.dpad_left){
                //scoreBasket();
                slidePosition = SlidePosition.FirstBasket;
            } else if (gamepad2.dpad_up) {
                //scoreBasket();
                slidePosition = SlidePosition.SecondBasket;
            } else if (gamepad2.dpad_right) {
                slidePosition = SlidePosition.Clipper;
            }

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

    private void SetSlidePosition(double newPosition){
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
    private double getSlidePosition(){
        return slideVL.getCurrentPosition();
    }
}