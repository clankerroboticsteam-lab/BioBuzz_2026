
package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.ArrayList;
import java.util.List;

@TeleOp(name = "Limelight Turret Test")
public class LimelightTurretTest extends LinearOpMode {

    private Limelight3A limelight;

    // AprilTag IDs for the cluster we want
    private static final int MIN_TAG_ID = 34;
    private static final int MAX_TAG_ID = 37;

    // Stores information about each detected tag
    public static class TagReading {
        public int id;

        public double distanceInches;
        public double horizontalDistanceInches;

        public double bearingDegrees;
        public double verticalAngleDegrees;

        public TagReading(int id, double distance,
                          double horizontalDistance,
                          double bearing, double verticalAngle) {

            this.id = id;
            this.distanceInches = distance;
            this.horizontalDistanceInches = horizontalDistance;
            this.bearingDegrees = bearing;
            this.verticalAngleDegrees = verticalAngle;
        }
    }

    @Override
    public void runOpMode() {

        // Initialize Limelight
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        // Set polling rate
        limelight.setPollRateHz(100);

        // Start Limelight
        limelight.start();

        telemetry.addLine("Limelight Initialized");
        telemetry.addLine("Looking for AprilTags 34-37");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            LLResult result = limelight.getLatestResult();

            if (result != null && result.isValid()) {

                // Get measurements for all visible tags in cluster 34-37
                List<TagReading> readings = getClusterReadings(result);

                telemetry.addData("Pipeline", result.getPipelineIndex());

                if (readings.isEmpty()) {

                    telemetry.addLine("No cluster tags detected");

                } else {

                    telemetry.addData("Tags Found", readings.size());

                    for (TagReading tag : readings) {

                        telemetry.addLine("--------------------");

                        telemetry.addData("AprilTag ID", tag.id);

                        telemetry.addData(
                                "3D Distance",
                                "%.2f inches",
                                tag.distanceInches
                        );

                        telemetry.addData(
                                "Horizontal Distance",
                                "%.2f inches",
                                tag.horizontalDistanceInches
                        );

                        telemetry.addData(
                                "Turret Bearing",
                                "%.2f degrees",
                                tag.bearingDegrees
                        );

                        telemetry.addData(
                                "Vertical Angle",
                                "%.2f degrees",
                                tag.verticalAngleDegrees
                        );
                    }
                }

            } else {
                telemetry.addLine("Waiting for Limelight data...");
            }

            telemetry.update();
        }

        limelight.stop();
    }

    // Reads all visible AprilTags in cluster 34-37
    private List<TagReading> getClusterReadings(LLResult result) {

        List<TagReading> readings = new ArrayList<>();

        if (result == null || !result.isValid()) {
            return readings;
        }

        List<LLResultTypes.FiducialResult> fiducials =
                result.getFiducialResults();

        if (fiducials == null) {
            return readings;
        }

        for (LLResultTypes.FiducialResult fiducial : fiducials) {

            int id = fiducial.getFiducialId();

            // Only use AprilTags 34, 35, 36, and 37
            if (id < MIN_TAG_ID || id > MAX_TAG_ID) {
                continue;
            }

            // Get AprilTag position relative to camera
            Pose3D targetPose = fiducial.getTargetPoseCameraSpace();

            if (targetPose == null) {
                continue;
            }

            // Convert position to inches
            Position position =
                    targetPose.getPosition().toUnit(DistanceUnit.INCH);

            double x = position.x;
            double y = position.y;
            double z = position.z;

            if (z <= 0) {
                continue;
            }

            // Straight-line 3D distance
            double distance =
                    Math.sqrt(x * x + y * y + z * z);

            // Horizontal distance for turret/shooter calculations
            double horizontalDistance =
                    Math.sqrt(x * x + z * z);

            // Horizontal aiming angle
            double bearing =
                    Math.toDegrees(Math.atan2(x, z));

            // Vertical angle (positive means target is above camera)
            double verticalAngle =
                    Math.toDegrees(Math.atan2(-y, horizontalDistance));

            readings.add(new TagReading(
                    id,
                    distance,
                    horizontalDistance,
                    bearing,
                    verticalAngle
            ));
        }

        return readings;
    }
}
