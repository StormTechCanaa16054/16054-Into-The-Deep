package org.firstinspires.ftc.teamcode.roadRunner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name = "teste encoder", group = "Linear OpMode")
public class encoder extends LinearOpMode {
    private DcMotor slideVL,slideVR;

    @Override
    public void runOpMode() {
        slideVL = hardwareMap.get(DcMotor.class, "slideVL");
        slideVR = hardwareMap.get(DcMotor.class, "slideVR");


        slideVL.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideVR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("Encoder 1 Value", slideVL.getCurrentPosition());
            telemetry.addData("Encoder 2 Value", slideVR.getCurrentPosition());
            telemetry.update();
        }
    }
}
