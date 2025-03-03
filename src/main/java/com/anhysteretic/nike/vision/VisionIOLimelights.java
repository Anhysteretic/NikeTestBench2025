package com.anhysteretic.nike.vision;

import com.anhysteretic.nike.constants.RC;
import com.anhysteretic.nike.lib.limelight.LimelightHelpers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class VisionIOLimelights {
    NetworkTable charlieTable = NetworkTableInstance.getDefault().getTable(RC.Limelights.front);

    public VisionIOLimelights() {

    }

    private void setLLSettings() {
        charlieTable.getEntry("camerapose_robotspace_set").setDoubleArray(RC.Limelights.frontPose);

        var gyroAngle = robotState.getLatestFieldToRobot().getValue().getRotation();
        var gyroAngularVelocity = Units
                .radiansToDegrees(robotState.getLatestRobotRelativeChassisSpeed().omegaRadiansPerSecond);
        LimelightHelpers.SetRobotOrientation(RC.Limelights.frontName, gyroAngle.getDegrees(),
                gyroAngularVelocity, 0, 0, 0, 0);
    }

}
