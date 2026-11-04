package org.firstinspires.ftc.teamcode.LimeLight;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Limelight Test")
public class Test1 extends LinearOpMode {

    private Limelight3A limelight;

    @Override
    public void runOpMode() {

        limelight = hardwareMap.get(
                Limelight3A.class, "limelight"
        );

        limelight.pipelineSwitch(0);
        limelight.start();

        telemetry.addLine("Limelight Initialized");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                telemetry.addData("Target X", result.getTx());
                telemetry.addData("Target Y", result.getTy());
                telemetry.addData("Target Area", result.getTa());

            } else {
                telemetry.addLine("No Target Detected");
            }

            telemetry.update();
        }

        limelight.stop();
    }
}