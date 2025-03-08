package com.anhysteretic.nike;

import static edu.wpi.first.units.Units.*;

import com.anhysteretic.nike.constants.TunerConstants;
import com.anhysteretic.nike.drivetrain.Drivetrain;
import com.anhysteretic.nike.drivetrain.Snapping;
import com.anhysteretic.nike.vision.Vision;
import com.anhysteretic.nike.vision.VisionIO;
import com.anhysteretic.nike.vision.VisionIOLimelights;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.team254.vision.VisionFieldPoseEstimate;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

import java.util.function.Consumer;

public class RobotContainer {
  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  private double MaxAngularRate =
      RotationsPerSecond.of(0.75)
          .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(MaxSpeed * 0.1)
          .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
          .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final VisionIO visionIO;
  public final Vision vision;

  public final ChassisSpeeds testSpeeds = new ChassisSpeeds(MaxSpeed/2, 0, 0);

  public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer = new Consumer<VisionFieldPoseEstimate>(){
    @Override
    public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
      drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
    }
  };

  public RobotContainer() {
    this.visionIO = new VisionIOLimelights();
    this.vision = new Vision(visionIO, visionEstimateConsumer, drivetrain);
    configureBindings();
  }

  private void configureBindings() {

    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(
            () ->
                drive
                    .withVelocityX(
                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(
                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(
                        -joystick.getRightX()
                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
            ));

    joystick.a().whileTrue(new RunCommand(() -> drivetrain.setControl(drivetrain.test.withSpeeds(testSpeeds)), drivetrain));


//    this.joystick.rightBumper().whileTrue(new Snapping(drivetrain, true));
//    this.joystick.leftBumper().whileTrue(new Snapping(drivetrain, false));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
