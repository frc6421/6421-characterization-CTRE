// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;

public class DriveCommand extends Command {
  private final DriveSubsystem driveSubsystem;

  private final CommandXboxController driverController;

  private final SwerveRequest.FieldCentric driveRequest;

  private final SlewRateLimiter xDriveSlew;
  private final SlewRateLimiter yDriveSlew;

  // CONSTANTS \\
  private static final double PERCENT_DEADBAND = 0.1;

  private static final double MAX_SPEED = DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC;
  private static final double MAX_ANGULAR_RATE = 2 * Math.PI;

  /** Creates a new DriveCommand. */
  public DriveCommand(DriveSubsystem driveSubsystem, CommandXboxController controller) {

    this.driveSubsystem = driveSubsystem;
    driverController = controller;

    driveRequest = new SwerveRequest.FieldCentric()
        .withDeadband(MAX_SPEED * PERCENT_DEADBAND)
        .withRotationalDeadband(MAX_ANGULAR_RATE * PERCENT_DEADBAND) // Add a 10% deadband
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withSteerRequestType(SteerRequestType.MotionMagicExpo);

    xDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);
    yDriveSlew = new SlewRateLimiter(DriveConstants.DRIVE_SLEW_RATE);

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double xSpeed = xDriveSlew
        .calculate(-1 * driverController.getLeftY() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
    double ySpeed = yDriveSlew
        .calculate(-1 * driverController.getLeftX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);

    driveSubsystem.applyRequest(
        () -> driveRequest
            .withVelocityX(xSpeed)
            .withVelocityY(ySpeed)
            .withRotationalRate(-1 * driverController.getRightX() * DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // This is a default command so must be false
  @Override
  public boolean isFinished() {
    return false;
  }
}