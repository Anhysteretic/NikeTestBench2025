package com.anhysteretic.nike.vision;

import com.anhysteretic.nike.RobotState;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;
import com.team254.vision.VisionFieldPoseEstimate;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.Logger;

import java.util.Arrays;
import java.util.Optional;
import java.util.Set;
import java.util.stream.Collectors;

public class Vision extends SubsystemBase {
    private final VisionIO io;

    private final RobotState robotState;

    private final VisionIOInputsAutoLogged inputs = new VisionIOInputsAutoLogged();

    private double lastProcessedTimestamp;

    public Vision(VisionIO io, RobotState robotState){
        this.io = io;
        this.robotState = robotState;
    }

    @Override
    public void periodic() {

        io.updateInputs(this.inputs);
        Logger.processInputs("Vision", this.inputs);
         
    }

    public void updateVision(boolean cameraSeesTarget, FiducialObservation[] cameraFiducialObservations, MegatagPoseEstimate cameraMegatagPoseEstimate, MegatagPoseEstimate cameraMegatag2PoseEstimate){

        if (cameraMegatagPoseEstimate != null) {
            String logPreface = "Vision/Charlie";
            var updateTimestamp = cameraMegatagPoseEstimate.timestampSeconds;

            boolean alreadyProcessedTimestamp = lastProcessedTimestamp == updateTimestamp;

            if (!alreadyProcessedTimestamp && cameraSeesTarget){

                Optional<VisionFieldPoseEstimate> megatagEstimate = processMegatagPoseEstimate(cameraMegatagPoseEstimate);
                Optional<VisionFieldPoseEstimate> megatag2Estimate = processMegatag2PoseEstimate(cameraMegatag2PoseEstimate, logPreface);

                boolean used_megatag = false;

                if (megatagEstimate.isPresent()){
                    if (shouldUseMegatag(cameraMegatagPoseEstimate, cameraFiducialObservations,
                            logPreface)) {
                        Logger.recordOutput(logPreface + "MegatagEstimate",
                                megatagEstimate.get().getVisionRobotPoseMeters());
                        robotState.updateMegatagEstimate(megatagEstimate.get());
                        used_megatag = true;
                    } else {
                        Logger.recordOutput(logPreface + "MegatagEstimateRejected", megatagEstimate.get().getVisionRobotPoseMeters());
                    }
                }

                if (megatag2Estimate.isPresent() && !used_megatag) {
                    if (shouldUseMegatag2(cameraMegatag2PoseEstimate.timestampSeconds, logPreface)) {
                        Logger.recordOutput(logPreface + "Megatag2Estimate",
                                megatag2Estimate.get().getVisionRobotPoseMeters());
                        robotState.updateMegatagEstimate(megatag2Estimate.get());
                    } else {
                        if (megatagEstimate.isPresent()) {
                            Logger.recordOutput(logPreface + "Megatag2EstimateRejected",
                                    megatag2Estimate.get().getVisionRobotPoseMeters());
                        }
                    }
                }

                lastProcessedTimestamp = updateTimestamp;
            }
        }
    }

    private Optional<VisionFieldPoseEstimate> processMegatagPoseEstimate(MegatagPoseEstimate poseEstimate) {
        var loggedFieldToRobot = robotState.getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var fieldToRobotEstimate = poseEstimate.fieldToCamera;

        // distance from current pose to vision estimated pose
        double poseDifference = fieldToRobotEstimate.getTranslation()
                .getDistance(loggedFieldToRobot.get().getTranslation());

        if (poseEstimate.fiducialIds.length > 0) {
            double xyStds = 1.0;
            double degStds = 12;
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2) {
                xyStds = 0.5;
                degStds = 6;
            }
            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 1.0;
                degStds = 12;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 2.0;
                degStds = 30;
            }

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(degStds));
            return Optional.of(
                    new VisionFieldPoseEstimate(fieldToRobotEstimate, poseEstimate.timestampSeconds,
                            visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    private Optional<VisionFieldPoseEstimate> processMegatag2PoseEstimate(MegatagPoseEstimate poseEstimate, String logPreface) {
        var loggedFieldToRobot = robotState.getFieldToRobot(poseEstimate.timestampSeconds);
        if (loggedFieldToRobot.isEmpty()) {
            return Optional.empty();
        }

        var fieldToRobotEstimate = poseEstimate.fieldToCamera;

        // distance from current pose to vision estimated pose
        double poseDifference = fieldToRobotEstimate.getTranslation()
                .getDistance(loggedFieldToRobot.get().getTranslation());

        double xyStds;
        if (poseEstimate.fiducialIds.length > 0) {
            // multiple targets detected
            if (poseEstimate.fiducialIds.length >= 2 && poseEstimate.avgTagArea > 0.1) {
                xyStds = 0.2;
            }

            // 1 target with large area and close to estimated pose
            else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStds = 0.5;
            }
            // 1 target farther away and estimated pose is close
            else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStds = 1.0;
            } else if (poseEstimate.fiducialIds.length > 1) {
                xyStds = 1.2;
            } else {
                xyStds = 2.0;
            }

            Logger.recordOutput(logPreface + "Megatag2StdDev", xyStds);
            Logger.recordOutput(logPreface + "Megatag2AvgTagArea", poseEstimate.avgTagArea);
            Logger.recordOutput(logPreface + "Megatag2PoseDifference", poseDifference);

            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStds, xyStds, Units.degreesToRadians(50.0));
            fieldToRobotEstimate = new Pose2d(fieldToRobotEstimate.getTranslation(),
                    loggedFieldToRobot.get().getRotation());
            return Optional.of(
                    new VisionFieldPoseEstimate(fieldToRobotEstimate, poseEstimate.timestampSeconds,
                            visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    private boolean shouldUseMegatag(MegatagPoseEstimate poseEstimate, FiducialObservation[] fiducials, String logPreface) {
        final double kMinAreaForMegatagEnabled = 0.4;
        final double kMinAreaForMegatagDisabled = 0.05;

        double kMinAreaForMegatag = 0.0;
        if (DriverStation.isDisabled()) {
            kMinAreaForMegatag = kMinAreaForMegatagDisabled;
        } else {
            kMinAreaForMegatag = kMinAreaForMegatagEnabled;
        }

        final int kExpectedTagCount = 2;

        final double kLargeYawThreshold = Units.degreesToRadians(200.0);
        final double kLargeYawEventTimeWindowS = 0.05;

        var maxYawVel = robotState.getMaxAbsDriveYawAngularVelocityInRange(
                poseEstimate.timestampSeconds - kLargeYawEventTimeWindowS,
                poseEstimate.timestampSeconds);
        if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
            Logger.recordOutput("Vision/Elevator/MegatagYawAngular", false);
            return false;
        }
        Logger.recordOutput("Vision/Elevator/MegatagYawAngular", true);

        if (poseEstimate.avgTagArea < kMinAreaForMegatag) {
            Logger.recordOutput(logPreface + "megaTagAvgTagArea", false);
            return false;
        }
        Logger.recordOutput(logPreface + "megaTagAvgTagArea", true);

        if (poseEstimate.fiducialIds.length != kExpectedTagCount) {
            Logger.recordOutput(logPreface + "fiducialLength", false);
            return false;
        }
        Logger.recordOutput(logPreface + "fiducialLength", true);

        if (poseEstimate.fiducialIds.length < 1) {
            Logger.recordOutput(logPreface + "fiducialLengthLess1", false);
            return false;
        }
        Logger.recordOutput(logPreface + "fiducialLengthLess1", true);

        if (poseEstimate.fieldToCamera.getTranslation().getNorm() < 1.0) {
            Logger.recordOutput(logPreface + "NormCheck", false);
            return false;
        }
        Logger.recordOutput(logPreface + "NormCheck", true);

        for (var fiducial : fiducials) {
            if (fiducial.ambiguity > .9) {
                Logger.recordOutput(logPreface + "Ambiguity", false);
                return false;
            }
        }
        Logger.recordOutput(logPreface + "Ambiguity", true);

        return true;
    }

    private boolean shouldUseMegatag2(double timestamp, String preface) {
        final double kLargePitchRollYawEventTimeWindowS = 0.1;
        final double kLargeYawThreshold = Units.degreesToRadians(100.0);

        var maxYawVel = robotState.getMaxAbsDriveYawAngularVelocityInRange(
                timestamp - kLargePitchRollYawEventTimeWindowS,
                timestamp);
        if (maxYawVel.isPresent() && Math.abs(maxYawVel.get()) > kLargeYawThreshold) {
            Logger.recordOutput(preface + "PinholeYawAngular", false);
            return false;
        }
        Logger.recordOutput(preface + "PinholeYawAngular", true);

        return true;
    }
}
