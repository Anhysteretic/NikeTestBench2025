package com.anhysteretic.nike.drivetrain;

import static edu.wpi.first.units.Units.*;

import com.anhysteretic.nike.LimelightHelpers;
import com.anhysteretic.nike.constants.RC;
import com.anhysteretic.nike.constants.TunerConstants.TunerSwerveDrivetrain;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import org.littletonrobotics.junction.Logger;

import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements Subsystem so it can easily
 * be used in command-based projects.
 */
public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
  private double m_lastSimTime;

  /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
  /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean m_hasAppliedOperatorPerspective = false;

    public PIDController controller = new PIDController(0.5, 0, 0);
    public final SwerveRequest.ApplyRobotSpeeds m_robotSpeeds = new SwerveRequest.ApplyRobotSpeeds();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds =
            new SwerveRequest.ApplyRobotSpeeds();

    public boolean hasSeeded = false;
    Matrix<N3, N1> stdDevsVisionMT1 = VecBuilder.fill(0.7, 0.7, Math.PI);
    Matrix<N3, N1> stdDevsVisionMT2 = VecBuilder.fill(0.7, 0.7, 2*Math.PI);

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable networkTablePoses = inst.getTable("Drivetrain Poses");

    StructPublisher<Pose2d> mt1 =
            networkTablePoses.getStructTopic("Megatag 1", Pose2d.struct).publish();
    StructPublisher<Pose2d> mt2 =
            networkTablePoses.getStructTopic("Megatag 2", Pose2d.struct).publish();
    StructPublisher<Pose2d> ctre = networkTablePoses.getStructTopic("CTRE", Pose2d.struct).publish();

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants     Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
     *                                0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param modules                 Constants for each specific module
     */
    public Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    /**
     * Constructs a CTRE SwerveDrivetrain using the specified constants.
     *
     * <p>This constructs the underlying hardware devices, so users should not construct the devices
     * themselves. If they need the devices, they can access them through getters in the classes.
     *
     * @param drivetrainConstants       Drivetrain-wide constants for the swerve drive
     * @param odometryUpdateFrequency   The frequency to run the odometry loop. If unspecified or set to
     *                                  0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
     * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
     *                                  [x, y, theta]ᵀ, with units in meters and radians
     * @param visionStandardDeviation   The standard deviation for vision calculation in the form [x, y,
     *                                  theta]ᵀ, with units in meters and radians
     * @param modules                   Constants for each specific module
     */
    public Drivetrain(
            SwerveDrivetrainConstants drivetrainConstants,
            double odometryUpdateFrequency,
            Matrix<N3, N1> odometryStandardDeviation,
            Matrix<N3, N1> visionStandardDeviation,
            SwerveModuleConstants<?, ?, ?>... modules) {
        super(
                drivetrainConstants,
                odometryUpdateFrequency,
                odometryStandardDeviation,
                visionStandardDeviation,
                modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }

        configureAutoBuilder();
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                    () -> getState().Pose, // Supplier of current robot pose
                    this::resetPose, // Consumer for seeding pose against auto
                    () -> getState().Speeds, // Supplier of current robot speeds
                    // Consumer of ChassisSpeeds and feedforwards to drive the robot
                    (speeds, feedforwards) ->
                            setControl(
                                    m_pathApplyRobotSpeeds
                                            .withSpeeds(speeds)
                                            .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                                            .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())),
                    new PPHolonomicDriveController(
                            // PID constants for translation
                            new PIDConstants(10, 0, 0),
                            // PID constants for rotation
                            new PIDConstants(7, 0, 0)),
                    config,
                    // Assume the path needs to be flipped for Red vs Blue, this is normally the case
                    () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                    this // Subsystem for requirements
            );
        } catch (Exception ex) {
            DriverStation.reportError(
                    "Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    /**
     * Returns a command that applies the specified control request to this swerve drivetrain.
     *
     * @param request Function returning the request to apply
     * @return Command to run
     */
    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    @Override
    public void periodic() {

        if (DriverStation.isDisabled()) {
            if (!m_hasAppliedOperatorPerspective) {
                DriverStation.getAlliance()
                        .ifPresent(
                                allianceColor -> {
                                    setOperatorPerspectiveForward(
                                            allianceColor == Alliance.Red
                                                    ? kRedAlliancePerspectiveRotation
                                                    : kBlueAlliancePerspectiveRotation);
                                    m_hasAppliedOperatorPerspective = true;
                                });
            }

            LimelightHelpers.PoseEstimate poseEstimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Limelights.front);
            if (poseEstimate == null) {
                return;
            }

            if (poseEstimate.tagCount >= 2) {
                this.setVisionMeasurementStdDevs(this.stdDevsVisionMT1);
                // remember to unflip for nemo
                var pose = new Pose2d(poseEstimate.pose.getTranslation(), new Rotation2d(poseEstimate.pose.getRotation().getRadians() + Math.PI));
                this.addVisionMeasurement(pose, poseEstimate.timestampSeconds);
                LimelightHelpers.SetRobotOrientation(
                        RC.Limelights.front, poseEstimate.pose.getRotation().getDegrees(), 0, 0, 0, 0, 0);
                if (!hasSeeded) {
                    this.hasSeeded = true;
                }
            }
        } else {
            LimelightHelpers.PoseEstimate poseEstimate =
                    LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Limelights.front);
            if (poseEstimate == null) {
                return;
            }
            LimelightHelpers.SetRobotOrientation(
                    RC.Limelights.front, this.getPigeon2().getYaw().getValueAsDouble(), 0, 0, 0, 0, 0);
            if (poseEstimate.tagCount >= 2) {
                this.setVisionMeasurementStdDevs(this.stdDevsVisionMT2);
                var pose = new Pose2d(poseEstimate.pose.getTranslation(), new Rotation2d(poseEstimate.pose.getRotation().getRadians() + Math.PI));
                this.addVisionMeasurement(pose, poseEstimate.timestampSeconds);
            }
        }

        var one = LimelightHelpers.getBotPoseEstimate_wpiBlue(RC.Limelights.front);
        var two = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(RC.Limelights.front);
        if (one != null) {
            this.mt1.set(one.pose);
        }
        if (two != null) {
            this.mt2.set(two.pose);
        }
        this.ctre.set(this.getState().Pose);

        SmartDashboard.putNumber("TargetSpeed", test.Speeds.vxMetersPerSecond);
        SmartDashboard.putNumber("current speed", this.getState().Speeds.vxMetersPerSecond);
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier =
                new Notifier(
                        () -> {
                            final double currentTime = Utils.getCurrentTimeSeconds();
                            double deltaTime = currentTime - m_lastSimTime;
                            m_lastSimTime = currentTime;

                            /* use the measured time delta, get battery voltage from WPILib */
                            updateSimState(deltaTime, RobotController.getBatteryVoltage());
                        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision camera.
     * @param timestampSeconds      The timestamp of the vision measurement in seconds.
     */
    @Override
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        super.addVisionMeasurement(visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds));
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>Note that the vision measurement standard deviations passed into this method will continue
     * to apply to future measurements until a subsequent call to {@link
     * #setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in seconds.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose measurement in the form
     *                                 [x, y, theta]ᵀ, with units in meters and radians.
     */
    @Override
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        super.addVisionMeasurement(
                visionRobotPoseMeters, Utils.fpgaToCurrentTime(timestampSeconds), visionMeasurementStdDevs);
    }

    public SwerveRequest.ApplyRobotSpeeds test = new SwerveRequest.ApplyRobotSpeeds();

}
