package com.anhysteretic.nike.constants;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

public class RC {
  public static Supplier<Boolean> isRedAlliance =
      () -> {
        var alliance = DriverStation.getAlliance();
        return alliance.filter(value -> value == DriverStation.Alliance.Red).isPresent();
      };

  public static final double LOOKBACK_TIME = 1.0;

  public static final RunType robotType = RunType.COMP;

  public static final CANBus kCANBus = new CANBus("canivore2", "./logs/example.hoot");

  public class Limelights {
    public static String frontName = "limelight-charlie";
    public static String backName = "limelight-gamma";
    // x y z roll pitch yaw
    public static double[] frontPose = {0.0, 0.0, 0.25, 180, 0.0, 0.0};
  }

  public enum RunType {
    SIM, // Simulation
    DEV, // Developer-tuning mode
    COMP, // Comp code, real robot code
    REPLAY
  }
}
