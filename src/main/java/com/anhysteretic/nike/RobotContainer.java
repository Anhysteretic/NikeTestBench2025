package com.anhysteretic.nike;

import static edu.wpi.first.units.Units.*;

import com.anhysteretic.nike.constants.TunerConstants;
import com.anhysteretic.nike.drivetrain.DriveMaintainHeading;
import com.anhysteretic.nike.drivetrain.Drivetrain;
import com.anhysteretic.nike.drivetrain.Snapping;
import com.anhysteretic.nike.vision.Vision;
import com.anhysteretic.nike.vision.VisionIO;
import com.anhysteretic.nike.vision.VisionIOLimelights;
import com.pathplanner.lib.auto.AutoBuilder;
import com.team254.vision.VisionFieldPoseEstimate;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.Consumer;

public class RobotContainer {

  private double MaxSpeed =
      TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final Drivetrain drivetrain = TunerConstants.createDrivetrain();

  public final VisionIO visionIO;
  public final Vision vision;

  public final ChassisSpeeds testSpeeds = new ChassisSpeeds(MaxSpeed / 2, 0, 0);

  private final SendableChooser<Command> autoChooser;

  public final Consumer<VisionFieldPoseEstimate> visionEstimateConsumer =
      new Consumer<VisionFieldPoseEstimate>() {
        @Override
        public void accept(VisionFieldPoseEstimate visionFieldPoseEstimate) {
          drivetrain.addVisionMeasurement(visionFieldPoseEstimate);
        }
      };

  private final DriveMaintainHeading driveMaintainHeading;

  public RobotContainer() {
    this.visionIO = new VisionIOLimelights();
    this.vision = new Vision(visionIO, visionEstimateConsumer, drivetrain);
    this.driveMaintainHeading =
        new DriveMaintainHeading(
            drivetrain,
            () -> -joystick.getLeftY(),
            () -> -joystick.getLeftX(),
            () -> -joystick.getRightX());
    configureBindings();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(driveMaintainHeading);

    //    drivetrain.setDefaultCommand(
    //        // Drivetrain will execute this command periodically
    //        drivetrain.applyRequest(
    //            () ->
    //                drive
    //                    .withVelocityX(
    //                        -joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y
    // (forward)
    //                    .withVelocityY(
    //                        -joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
    //                    .withRotationalRate(
    //                        -joystick.getRightX()
    //                            * MaxAngularRate) // Drive counterclockwise with negative X (left)
    //            ));

    joystick
        .a()
        .whileTrue(
            new RunCommand(
                () -> drivetrain.setControl(drivetrain.test.withSpeeds(testSpeeds)), drivetrain));

    this.joystick.rightBumper().whileTrue(new Snapping(drivetrain, true));
    this.joystick.leftBumper().whileTrue(new Snapping(drivetrain, false));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //    return Commands.print("No autonomous command configured");
  }
}
