// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VelocityFeedforwardCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private ChassisSpeeds setChassisSpeeds;

  private final PointWheelsAt zeroWheelsRequest;
  private final ApplyRobotSpeeds driveVelocityRequest;
  private final ApplyRobotSpeeds stopRobotRequest;

  /**
   * The velocity to drive the robot in this command.
   * </p>
   * Can be positive or negative but should be less than max robot velocity
   */
  private double setRobotVelocity;
  private double finalRobotVelocity;

  // Used to verify voltage and velocity are consistant between modules.
  private double[] moduleVoltage;
  private double[] moduleVelocity;

  private double averageVoltage;

  /**
   * Used to determine the feed forward voltage needed to get the robot moving
   * close to the set velocity.
   * </p>
   * This should be run at several different velocities, both froward and
   * backward, to verify that the data appears good. kV should be fairly linear
   * over the range of velocities.
   * </p>
   * When done update the {@link DriveSubsystem#kV} value of the Drive Motor
   * {@link DriveSubsystem#DRIVE_GAINS}.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VelocityFeedforwardCommand(CommandSwerveDrivetrain drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    setRobotVelocity = 0;

    moduleVoltage = new double[4];
    moduleVelocity = new double[4];

    // Swerve request to make sure the wheels point in the x direction
    zeroWheelsRequest = new PointWheelsAt();
    // Swerve request to use to drive the robot
    driveVelocityRequest = new ApplyRobotSpeeds();
    // Swerve request to stop the robot
    stopRobotRequest = new ApplyRobotSpeeds();

    Shuffleboard.getTab("4: Velocity Feedforward").add(this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry
    driveSubsystem.seedFieldCentric();
    driveSubsystem.tareEverything();
    // Set wheels to face forward.
    driveSubsystem.setControl(zeroWheelsRequest);

    // Set moduleVoltage and moduleVelocity array values with intitial voltage and
    // velocity (should be zero)
    for (int i = 0; i < 4; ++i) {
      moduleVoltage[i] = getModuleVoltage(i);
      moduleVelocity[i] = getModuleVelocity(i);
    }

    averageVoltage = 0;

    // Set robot velocity (should be zero)
    finalRobotVelocity = getRobotVelocity();

    // Set chassis speed based on the velocity set in Shuffleboard
    setChassisSpeeds = new ChassisSpeeds(setRobotVelocity, 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    var sumVoltage = 0.0;
    // Get voltage and velocity values for each module drive motor
    for (int i = 0; i < 4; ++i) {
      sumVoltage += moduleVoltage[i] = getModuleVoltage(i);
      moduleVelocity[i] = getModuleVelocity(i);
    }

    averageVoltage = sumVoltage / 4;

    finalRobotVelocity = getRobotVelocity();

    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getState().Pose.getX()) > 3.5;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  public void setSetRobotVelocity(double velocity) {
    this.setRobotVelocity = MathUtil.clamp(velocity, -1 * TunerConstants.kSpeedAt12Volts.magnitude(),
        TunerConstants.kSpeedAt12Volts.magnitude());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Set X Velocity", () -> setRobotVelocity, this::setSetRobotVelocity);
    builder.addDoubleProperty("Module 0 Voltage", () -> getModuleVoltage(0), null);
    builder.addDoubleProperty("Module 0 Velocity", () -> getModuleVelocity(0), null);
    builder.addDoubleProperty("Module 1 Voltage", () -> getModuleVoltage(1), null);
    builder.addDoubleProperty("Module 1 Velocity", () -> getModuleVelocity(1), null);
    builder.addDoubleProperty("Module 2 Voltage", () -> getModuleVoltage(2), null);
    builder.addDoubleProperty("Module 2 Velocity", () -> getModuleVelocity(2), null);
    builder.addDoubleProperty("Module 3 Voltage", () -> getModuleVoltage(3), null);
    builder.addDoubleProperty("Module 3 Velocity", () -> getModuleVelocity(3), null);
    builder.addDoubleProperty("Average Module Voltage", () -> averageVoltage, null);
    builder.addDoubleProperty("Final Robot Velocity", () -> finalRobotVelocity, null);
    builder.addDoubleProperty("kV (voltage - velocity)", () -> averageVoltage / finalRobotVelocity, null);
  }

  private double getModuleVoltage(int module) {
    return driveSubsystem.getModule(module).getDriveMotor().getMotorVoltage().getValueAsDouble();
  }

  // TODO verify module units: rpm or m/s?
  private double getModuleVelocity(int module) {
    return driveSubsystem.getModule(module).getDriveMotor().getVelocity().getValueAsDouble();
  }

  /**
   * @return robot velocity in meters per second
   */
  private double getRobotVelocity() {
    return driveSubsystem.getKinematics()
        .toChassisSpeeds(driveSubsystem.getState().ModuleStates).vxMetersPerSecond;
  }
}
