package com.anhysteretic.nike.vision;

import com.anhysteretic.nike.RobotState;
import com.anhysteretic.nike.constants.RC;
import com.anhysteretic.nike.lib.limelight.LimelightHelpers;
import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelights implements VisionIO{
    NetworkTable charlieTable = NetworkTableInstance.getDefault().getTable(RC.Limelights.front);

    RobotState robotState;

    public VisionIOLimelights(RobotState robotState) {
        this.robotState = robotState;
        setLLSettings();
    }

    private void setLLSettings() {
        charlieTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Limelights.frontPose);

        var gyroAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
        var gyroAngularVelocity = Units
                .radiansToDegrees(robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(RC.Limelights.frontName, gyroAngle.getDegrees(),
                gyroAngularVelocity, 0, 0, 0, 0);
    }

    @Override
    public void updateInputs(VisionIOInputs inputs) {
        inputs.charlieSeesTarget = charlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.charlieSeesTarget){
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Limelights.frontName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Limelights.frontName);

            inputs.charlieMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.charlieMegatagCount = megatag.tagCount;
            inputs.charlieMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.charlieFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }

        setLLSettings();
    }

    @Override
    public void pollNetworktables() {
        VisionIOInputs inputs = new VisionIOInputs();

        inputs.charlieSeesTarget = charlieTable.getEntry("tv").getDouble(0) == 1.0;
        if (inputs.charlieSeesTarget){
            var megatag = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Limelights.frontName);
            var megatag2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Limelights.frontName);

            inputs.charlieMegatagPoseEstimate = MegatagPoseEstimate.fromLimelight(megatag);
            inputs.charlieMegatagCount = megatag.tagCount;
            inputs.charlieMegatag2PoseEstimates = MegatagPoseEstimate.fromLimelight(megatag2);
            inputs.charlieFiducialObservations = FiducialObservation.fromLimelight(megatag.rawFiducials);
        }
        setLLSettings();
    }
}
