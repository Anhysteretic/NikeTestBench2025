package com.anhysteretic.nike;

import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import com.team254.lib.ConcurrentTimeInterpolatableBuffer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class RobotState {

    public final static double LOOKBACK_TIME = 1.0;
    
    private final ConcurrentTimeInterpolatableBuffer<Pose2d> robotPose = ConcurrentTimeInterpolatableBuffer.createBuffer(LOOKBACK_TIME);
    private final AtomicReference<ChassisSpeeds> measuredRobotRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> measuredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> desiredFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());
    private final AtomicReference<ChassisSpeeds> fusedFieldRelativeChassisSpeeds = new AtomicReference<>(
            new ChassisSpeeds());

    public Map.Entry<Double, Pose2d> getLatestFieldToRobot() {
        return robotPose.getLatest();
    }

    public ChassisSpeeds getLatestMeasuredFieldRelativeChassisSpeeds() {
        return measuredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestRobotRelativeChassisSpeed() {
        return measuredRobotRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestDesiredFieldRelativeChassisSpeed() {
        return desiredFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedFieldRelativeChassisSpeed() {
        return fusedFieldRelativeChassisSpeeds.get();
    }

    public ChassisSpeeds getLatestFusedRobotRelativeChassisSpeed() {
        var speeds = getLatestRobotRelativeChassisSpeed();
        speeds.omegaRadiansPerSecond = getLatestFusedFieldRelativeChassisSpeed().omegaRadiansPerSecond;
        return speeds;
    }
}
