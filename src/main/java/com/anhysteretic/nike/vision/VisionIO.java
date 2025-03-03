package com.anhysteretic.nike.vision;

import com.anhysteretic.nike.vision.lib254.FiducialObservation;
import com.anhysteretic.nike.vision.lib254.MegatagPoseEstimate;
import org.littletonrobotics.junction.AutoLog;

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
