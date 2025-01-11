// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;


import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TuneCurrentLimitCommand extends Command {

  private final CommandSwerveDrivetrain driveSubsystem;

  private final ChassisSpeeds velocityDelta;
  private ChassisSpeeds setChassisSpeeds;

  private final PointWheelsAt zeroWheelRequest;
  private final ApplyRobotSpeeds stopRobotRequest;
  private final ApplyRobotSpeeds driveByChassisSpeedsRequest;

  private double[] moduleStatorCurrent;
  private double[] moduleSupplyCurrent;
  
  // Constants \\
  // TODO determine the velocity to set which means the wheels are slipping.
  /** In meters per second */
  private static final double VELOCITY_LIMIT = 0.1;

  /**
   * Used to find the current at which wheels start slipping.
   * </p>
   * To use place robot against wall or other unmovable barrier and start command.
   * </p>
   * When done update the value in {@link DriveSubsystem#SLIP_CURRENT_AMPS}
   * </p>
   * *** May need to be updated as treads wear and/or are replaced.
   * 
   * @param drive the drive subsystem used with command.
   */
  public TuneCurrentLimitCommand(CommandSwerveDrivetrain drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // The change in velocity to add each robot cycle.
    velocityDelta = new ChassisSpeeds(0.001, 0, 0);

    setChassisSpeeds = new ChassisSpeeds();

    // Swerve request to make sure the wheels point in the x direction
    zeroWheelRequest = new PointWheelsAt();
    // Swerve request to stop the robot
    stopRobotRequest = new ApplyRobotSpeeds();
    // Swerve request to use to drive the robot
    driveByChassisSpeedsRequest = new ApplyRobotSpeeds();

    moduleStatorCurrent = new double[4];
    moduleSupplyCurrent = new double[4];

    // Add command to shuffleboard.
    Shuffleboard.getTab("2: Current Limits").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set chassis speeds to 0 each time command starts
    setChassisSpeeds = new ChassisSpeeds();

    // Make sure wheels are pointed forward (x direction).
    // can add "".withModuleDirection(Rotation2D)" to turn all modules to a specific
    // angle if other directions want to be tested
    driveSubsystem.setControl(zeroWheelRequest);

    // Set moduleStatorCurrent and moduleSupplyCurrent array values with intitial
    // currents (should be zero)
    for (int i = 0; i < 4; ++i) {
      moduleStatorCurrent[i] = getmoduleStatorCurrent(i);
      moduleSupplyCurrent[i] = getModuleSupplyCurrent(i);
    }

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start driving forward
    driveSubsystem.setControl(driveByChassisSpeedsRequest.withSpeeds(setChassisSpeeds));

    // Get the current stator and supply currents for each module's drive motor
    for (int i = 0; i < 4; ++i) {
      moduleStatorCurrent[i] = getmoduleStatorCurrent(i);
      moduleSupplyCurrent[i] = getModuleSupplyCurrent(i);
    }

    // Increament the X velocity.
    setChassisSpeeds = setChassisSpeeds.plus(velocityDelta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Get the final stator and supply currents for each module's drive motor
    for (int i = 0; i < 4; ++i) {
      moduleStatorCurrent[i] = getmoduleStatorCurrent(i);
      moduleSupplyCurrent[i] = getModuleSupplyCurrent(i);
    }

    // Stop the motors.
    driveSubsystem.setControl(stopRobotRequest);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop command if all wheels are slipping
    return areWheelsSlipping();
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Set X velocity", () -> setChassisSpeeds.vxMetersPerSecond, null);
    builder.addDoubleProperty("Module 0 Stator Current", () -> moduleStatorCurrent[0], null);
    builder.addDoubleProperty("Module 0 Supply Current", () -> moduleSupplyCurrent[0], null);
    builder.addDoubleProperty("Module 1 Stator Current", () -> moduleStatorCurrent[1], null);
    builder.addDoubleProperty("Module 1 Supply Current", () -> moduleSupplyCurrent[1], null);
    builder.addDoubleProperty("Module 2 Stator Current", () -> moduleStatorCurrent[2], null);
    builder.addDoubleProperty("Module 2 Supply Current", () -> moduleSupplyCurrent[2], null);
    builder.addDoubleProperty("Module 3 Stator Current", () -> moduleStatorCurrent[3], null);
    builder.addDoubleProperty("Module 3 Supply Current", () -> moduleSupplyCurrent[3], null);
  }

  /**
   * Determine if all wheels are slipping. A wheel is slipping if the velocity is
   * higher than {@link TuneCurrentLimitCommand#VELOCITY_LIMIT}.
   * 
   * @return true if all wheels slipping, otherwise false
   */
  private boolean areWheelsSlipping() {
    // Cycle through all module drive motors to see if all wheels are slipping.
    for (int i = 0; i < 4; ++i) {
      if (driveSubsystem.getModule(i).getCurrentState().speedMetersPerSecond <= VELOCITY_LIMIT) {
        return false;
      }
    }
    return true;
  }

  private double getmoduleStatorCurrent(int module) {
    return driveSubsystem.getModule(module).getDriveMotor().getStatorCurrent().getValueAsDouble();
  }

  private double getModuleSupplyCurrent(int module) {
    return driveSubsystem.getModule(module).getDriveMotor().getSupplyCurrent().getValueAsDouble();
  }
}
