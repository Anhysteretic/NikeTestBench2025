package com.anhysteretic.nike.drivetrain;

import com.anhysteretic.nike.constants.RC;
import com.anhysteretic.nike.lib.limelight.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class Snapping extends Command {

  public Snapping(Drivetrain drivetrain, boolean goLeft) {
    this.drivetrain = drivetrain;

    this.setpoint = !goLeft ? -16.66 : 20.47;

    addRequirements(this.drivetrain);
    setName("Snapping");
  }

  private final Drivetrain drivetrain;

  private double setpoint;

  @Override
  public void initialize() {
    return;
  }

  @Override
  public void execute() {

    SmartDashboard.putNumber("Setpoint", setpoint);
    double speed =
        this.drivetrain.controller.calculate(
            LimelightHelpers.getTX(RC.Limelights.frontName), setpoint);
    SmartDashboard.putNumber("NOT Clamped Output", speed);
    speed = speed >= 0 ? Math.max(0, Math.min(speed, 8.5)) : -Math.max(0, Math.min(-speed, 8.5));
    speed /= 8.5;
    SmartDashboard.putNumber("Clamped Ouput", speed);
    this.drivetrain.setControl(
        this.drivetrain.m_robotSpeeds.withSpeeds(new ChassisSpeeds(0, speed, 0)));
  }

  @Override
  public boolean isFinished() {
    return this.drivetrain.controller.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    this.drivetrain.controller.reset();
  }
}
