// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPositionCommand extends SequentialCommandGroup {
  private SwerveDriveKinematics kinematics;

  /** Creates a new DriveToPositionCommand. */
  public DriveToPositionCommand(DriveSubsystem driveSubsystem) {

    kinematics = driveSubsystem.getSwerveDriveKinematics();

    addRequirements(driveSubsystem);

    TrajectoryConfig forwardConfig = new TrajectoryConfig(
        AutoConstants.AUTO_MAX_VELOCITY_METERS_PER_SECOND,
        AutoConstants.AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
        .setKinematics(kinematics);

    TrajectoryConfig reverseConfig = forwardConfig.setReversed(true);

    Trajectory driveToTwoOne = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(2, 1, new Rotation2d(Units.degreesToRadians(180)))), forwardConfig);

    Trajectory driveToZeroZero = TrajectoryGenerator.generateTrajectory(List.of(
        new Pose2d(2, 1, new Rotation2d(Units.degreesToRadians(180))),
        new Pose2d(0, 0, new Rotation2d(Units.degreesToRadians(180)))), reverseConfig);

    var thetaController = new ProfiledPIDController(
        AutoConstants.THETA_P, AutoConstants.THETA_I, AutoConstants.THETA_D,
        new TrapezoidProfile.Constraints(AutoConstants.AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC,
            AutoConstants.AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC));
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    HolonomicDriveController holonomicDriveController = new HolonomicDriveController(
        // Position controllers
        new PIDController(AutoConstants.X_DRIVE_P, AutoConstants.X_DRIVE_I, AutoConstants.X_DRIVE_D),
        new PIDController(AutoConstants.Y_DRIVE_P, AutoConstants.Y_DRIVE_I, AutoConstants.Y_DRIVE_D),
        thetaController);

    SwerveControllerCommand driveToFirstPosition = new SwerveControllerCommand(
        driveToTwoOne,
        driveSubsystem::getPose2d,
        kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    SwerveControllerCommand driveToScoreCommand = new SwerveControllerCommand(
        driveToZeroZero,
        driveSubsystem::getPose2d,
        kinematics,
        holonomicDriveController,
        driveSubsystem::autoSetModuleStates,
        driveSubsystem);

    addCommands(
        new InstantCommand(() -> driveSubsystem.seedFieldRelative(driveToTwoOne.getInitialPose())),
        driveToFirstPosition,
        new WaitCommand(5), /* TODO print Pose */
        new InstantCommand(() -> System.out.println(driveSubsystem.getState().Pose.toString())),
        driveToScoreCommand,
        new InstantCommand(() -> driveSubsystem.setControl(new SwerveRequest.ApplyChassisSpeeds())));
        // TODO print Pose
  }
}
