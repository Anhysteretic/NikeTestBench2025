package com.anhysteretic.nike.vision;

import org.littletonrobotics.junction.AutoLog;

import com.team254.vision.FiducialObservation;
import com.team254.vision.MegatagPoseEstimate;

public interface VisionIO {
    @AutoLog
    class VisionIOInputs{
        public boolean charlieSeesTarget;

        public FiducialObservation[] charlieFiducialObservations;

        public MegatagPoseEstimate charlieMegatagPoseEstimate;

        public int charlieMegatagCount;

        public MegatagPoseEstimate charlieMegatag2PoseEstimates;
    }

    void updateInputs(VisionIOInputs inputs);

    void pollNetworktables();
}
