// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VerifyOdometryCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private final SwerveRequest.PointWheelsAt zeroWheels;

  private boolean hasStatusCodeError;

  /**
   * Verify that odometry calcualtion is same as actual distance travelled.
   * </p>
   * If odometry is off by more than a acceptable range for the team, it is
   * suggested that the wheel be physically measured with a caliper and the
   * measured value entered into {@link DriveSubsystem#WHEEL_RADIUS_INCHES}.
   * </p>
   * After the value is changed the command should be run again to verify the
   * odometry is within the acceptable range.
   * 
   * @param driveSubsystem the drive subsystem used with command.
   */
  public VerifyOdometryCommand(CommandSwerveDrivetrain driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // Swerve request to make sure the wheels point in the x direction
    zeroWheels = new SwerveRequest.PointWheelsAt();

    // Add command to shuffleboard.
    Shuffleboard.getTab("1: Verify Odometry").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Point wheels forward (x direction)
    // can add "".withModuleDirection(Rotation2D)" to turn all modules to a specific
    // angle if other directions want to be tested
    driveSubsystem.setControl(zeroWheels);

    // Make current rotation forward.
    driveSubsystem.seedFieldCentric();
    // Reset odometry to 0,0
    driveSubsystem.tareEverything();

    // Set drive motors to coast. Check to see if successful
    hasStatusCodeError = (driveSubsystem.configNeutralMode(NeutralModeValue.Coast) == StatusCode.OK) ? false : true;
    if (hasStatusCodeError) {
      System.out.println("*** Set Neutral Mode FAILED initialize() ***");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Do nothing. Manually move the robot to distances.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Final robot distance (meters) :" + driveSubsystem.getState().Pose.getX());
    for (int i = 0; i < 4; ++i) {
      // TODO if these two print lines are the same remove the
      // getModuleDistanceMeters() method here and update in initSendable()
      System.out.println(
          "Module " + i + " [using getPosition(true)] distance (meters): "
              + driveSubsystem.getModule(i).getPosition(true).distanceMeters);
    }

    // Set drive motors to brake. Check to see if successful
    hasStatusCodeError = (driveSubsystem.configNeutralMode(NeutralModeValue.Brake) == StatusCode.OK) ? false : true;
    if (hasStatusCodeError) {
      System.out.println("*** Set Neutral Mode FAILED end() ***");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasStatusCodeError;
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Robot Distance (meters)", () -> driveSubsystem.getStateCopy().Pose.getX(), null);
    builder.addDoubleProperty("Module 0 Distance (meters)",
        () -> driveSubsystem.getModule(0).getPosition(true).distanceMeters, null);
    builder.addDoubleProperty("Module 1 Distance (meters)",
        () -> driveSubsystem.getModule(0).getPosition(true).distanceMeters, null);
    builder.addDoubleProperty("Module 2 Distance (meters)",
        () -> driveSubsystem.getModule(0).getPosition(true).distanceMeters, null);
    builder.addDoubleProperty("Module 3 Distance (meters)",
        () -> driveSubsystem.getModule(0).getPosition(true).distanceMeters, null);
  }
}
