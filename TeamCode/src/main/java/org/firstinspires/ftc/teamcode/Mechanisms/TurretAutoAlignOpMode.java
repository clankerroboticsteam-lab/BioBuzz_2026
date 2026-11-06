
package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.List;

@TeleOp(name = "Turret Auto Align Test", group = "Mechanisms")
public class TurretAutoAlignOpMode extends OpMode {

    private Limelight3A limelight3A;

    @Override
    public void init() {

        limelight3A = hardwareMap.get(
                Limelight3A.class, "limelight");

        limelight3A.setPollRateHz(100);
        limelight3A.start();

        telemetry.addLine("Limelight Initialized");
        telemetry.addLine("Looking for AprilTags 34-37");
        telemetry.update();
    }

    @Override
    public void loop() {

        LLResult result = limelight3A.getLatestResult();

        if (result != null && result.isValid()) {

            List<LLResultTypes.FiducialResult> fiducials =
                    result.getFiducialResults();

            boolean foundTag = false;

            for (LLResultTypes.FiducialResult fiducial : fiducials) {

                int id = fiducial.getFiducialId();

                // Only track AprilTags 34-37
                if (id < 34 || id > 37) {
                    continue;
                }

                foundTag = true;

                Pose3D targetPose =
                        fiducial.getTargetPoseCameraSpace();

                if (targetPose == null) {
                    continue;
                }

                Position position =
                        targetPose.getPosition().toUnit(DistanceUnit.INCH);

                double x = position.x;
                double y = position.y;
                double z = position.z;

                // Calculate straight-line distance
                double distance = Math.sqrt(
                        x * x + y * y + z * z
                );

                // Horizontal distance
                double horizontalDistance = Math.sqrt(
                        x * x + z * z
                );

                // Horizontal aiming angle
                double bearing = fiducial.getTargetXDegrees();

                // Vertical aiming angle
                double verticalAngle = fiducial.getTargetYDegrees();

                telemetry.addLine("--------------------");

                telemetry.addData("AprilTag ID", id);

                telemetry.addData(
                        "Distance", "%.2f inches", distance
                );

                telemetry.addData(
                        "Horizontal Distance",
                        "%.2f inches", horizontalDistance
                );

                telemetry.addData(
                        "Turret Bearing", "%.2f degrees", bearing
                );

                telemetry.addData(
                        "Vertical Angle", "%.2f degrees", verticalAngle
                );
            }

            if (!foundTag) {
                telemetry.addLine("No AprilTags 34-37 detected");
            }

        } else {
            telemetry.addLine("Waiting for Limelight...");
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        limelight3A.stop();
    }
}
