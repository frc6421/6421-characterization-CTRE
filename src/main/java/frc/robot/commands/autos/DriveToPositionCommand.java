// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import java.util.List;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.AutoConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToPositionCommand extends SequentialCommandGroup {
  private SwerveDriveKinematics kinematics;

  /** Creates a new DriveToPositionCommand. */
  public DriveToPositionCommand(CommandSwerveDrivetrain driveSubsystem) {

    kinematics = driveSubsystem.getKinematics();

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

    addCommands(
        Commands.runOnce(() -> driveSubsystem.resetPose(driveToTwoOne.getInitialPose()), driveSubsystem),
        driveSubsystem.runTrajectoryCommand(driveToTwoOne),
        Commands.waitSeconds(5),
        Commands.print(driveSubsystem.getState().Pose.toString()),
        driveSubsystem.runTrajectoryCommand(driveToZeroZero),
        Commands.runOnce(() -> driveSubsystem.applyRequest(() -> new ApplyRobotSpeeds()), driveSubsystem),
        Commands.print(driveSubsystem.getState().Pose.toString()));
  }
}
