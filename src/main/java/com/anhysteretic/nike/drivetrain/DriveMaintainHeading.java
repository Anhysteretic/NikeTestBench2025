package com.anhysteretic.nike.drivetrain;

import com.anhysteretic.nike.constants.RC;
import com.anhysteretic.nike.constants.TunerConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import org.littletonrobotics.junction.Logger;

import java.util.Optional;
import java.util.function.DoubleSupplier;

import static edu.wpi.first.units.Units.*;

public class DriveMaintainHeading extends Command {

    private double MaxSpeed =
            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate =
            RotationsPerSecond.of(0.75)
                    .in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    Drivetrain drivetrain;
    DoubleSupplier controlXSupplier, controlYSupplier, controlAngularVelocitySupplier;
    Optional<Rotation2d> mHeadingSetpoint;
    Double controlX, controlY, controlAngularVelocity;
    double kSteerJoystickDeadband = 0.1;
    double mJoystickLastTouched = -1;

    public DriveMaintainHeading(Drivetrain drivetrain, DoubleSupplier velocityX, DoubleSupplier velocityY, DoubleSupplier angularVelocity) {
        this.drivetrain = drivetrain;
        this.controlXSupplier = velocityX;
        this.controlYSupplier = velocityY;
        this.controlAngularVelocitySupplier = angularVelocity;

        driveHeading.HeadingController.setPID(3, 0, 0);
        driveHeading.HeadingController.enableContinuousInput(-Math.PI, Math.PI);

        addRequirements(this.drivetrain);
        setName("DriveMaintainHeading");
    }

    private final SwerveRequest.FieldCentric drive =
            new SwerveRequest.FieldCentric()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);

    private final SwerveRequest.FieldCentricFacingAngle driveHeading =
            new SwerveRequest.FieldCentricFacingAngle()
                    .withDeadband(MaxSpeed * 0.1)
                    .withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
                    .withDriveRequestType(SwerveModule.DriveRequestType.OpenLoopVoltage);


    @Override
    public void initialize() {
        mHeadingSetpoint = Optional.empty();
    }

    @Override
    public void execute() {
        this.controlX = this.controlXSupplier.getAsDouble();
        this.controlY = this.controlYSupplier.getAsDouble();
        this.controlAngularVelocity = this.controlAngularVelocitySupplier.getAsDouble();
        double velocityX = controlX * MaxSpeed;
        double velocityY = controlY * MaxSpeed;
        double angularVelocity = controlAngularVelocity * MaxAngularRate;
        double throttleFieldFrame = velocityX;//RC.isRedAlliance.get() ? -velocityX : velocityX;
        double strafeFieldFrame = velocityY;//RC.isRedAlliance.get() ? -velocityY : velocityY;
        if (Math.abs(controlAngularVelocity) > this.kSteerJoystickDeadband) {
            mJoystickLastTouched = Timer.getFPGATimestamp();
        }
        if (Math.abs(controlAngularVelocity) > kSteerJoystickDeadband
                || (epsilonEquals(mJoystickLastTouched, Timer.getFPGATimestamp(), 0.25)
                && Math.abs(drivetrain.getCurrentRobotSpeeds().omegaRadiansPerSecond) > Math
                .toRadians(10))) {
            drivetrain.setControl(drive.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withRotationalRate(angularVelocity));
            mHeadingSetpoint = Optional.empty();
            Logger.recordOutput("DriveMaintainHeading/Mode", "NoHeading");
        } else {
            if (mHeadingSetpoint.isEmpty()) {
                mHeadingSetpoint = Optional.of(drivetrain.getState().Pose.getRotation().rotateBy(Rotation2d.k180deg));
            }
            drivetrain.setControl(driveHeading.withVelocityX(throttleFieldFrame).withVelocityY(strafeFieldFrame)
                    .withTargetDirection(mHeadingSetpoint.get()));
            Logger.recordOutput("DriveMaintainHeading/Mode", "Heading");
            Logger.recordOutput("DriveMaintainHeading/HeadingSetpoint", mHeadingSetpoint.get().getDegrees());
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

}
