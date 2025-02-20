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

  public static final RunType robotType = RunType.COMP;

  public static final CANBus kCANBus = new CANBus("canivore2", "./logs/example.hoot");

  public class Limelights {
    public static String front = "limelight-charlie";
    public static String back = "limelight-gamma";
  }

  public enum RunType {
    SIM, // Simulation
    DEV, // Developer-tuning mode
    COMP, // Comp code, real robot code
    REPLAY
  }
}
