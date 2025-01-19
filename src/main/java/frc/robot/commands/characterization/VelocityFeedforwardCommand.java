// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VelocityFeedforwardCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private ChassisSpeeds setRobotSpeeds;

  private final ApplyRobotSpeeds driveVelocityRequest;
  private final ApplyRobotSpeeds stopRobotRequest;

  /* What to publish over networktables for static feedforward */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Static feedforward table publishers */
  private final NetworkTable velocityFeedForwardTable = inst.getTable("4 VelocityFeedforward");

  private final DoubleArrayPublisher moduleVoltage = velocityFeedForwardTable.getDoubleArrayTopic("Module Voltage").publish();
  private final DoubleArrayPublisher moduleVelocity = velocityFeedForwardTable.getDoubleArrayTopic("Module Velocity").publish();
  private final DoubleEntry averageVoltage = velocityFeedForwardTable.getDoubleTopic("Average Voltage").getEntry(0.0);
  private final DoublePublisher kV = velocityFeedForwardTable.getDoubleTopic("kV").publish();

  /**
   * Used to determine the feed forward voltage needed to get the robot moving
   * close to the set velocity.
   * </p>
   * This should be run at several different velocities, both froward and
   * backward, to verify that the data appears good. kV should be fairly linear
   * over the range of velocities.
   * </p>
   * When done update the {@link .withkV} value of the Drive Motor Gains in
   * {@link generated.TunerConstants#driveGains}.
   * 
   * @param drive the drive subsystem used with command.
   */
  public VelocityFeedforwardCommand(CommandSwerveDrivetrain drive) {
    driveSubsystem = drive;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    // Swerve request to use to drive the robot
    driveVelocityRequest = new ApplyRobotSpeeds();
    // Swerve request to stop the robot
    stopRobotRequest = new ApplyRobotSpeeds();

    SmartDashboard.putData("Velocity Feedforward Command", this);
    SmartDashboard.putNumber("4: Set Velocity", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Reset odometry
    driveSubsystem.seedFieldCentric();
    driveSubsystem.tareEverything();

    // Set moduleVoltage and moduleVelocity array values with intitial voltage and
    // velocity (should be zero)
      moduleVoltage.set(getModuleVoltage());
      moduleVelocity.set(getModuleVelocity());

    // Set chassis speed based on the velocity set in Shuffleboard
    setRobotSpeeds = new ChassisSpeeds(SmartDashboard.getNumber("4: Set Velocity", 0), 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setRobotSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    moduleVoltage.set(getModuleVoltage());
    moduleVelocity.set(getModuleVelocity());
    kV.set(averageVoltage.getAsDouble()/getRobotVelocity());

    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getState().Pose.getX()) > 3.5;
  }

  private double[] getModuleVoltage() {
    double voltageSum = 0.0;
    double[] voltage = new double[4];
    for (int i = 0; i < 4; ++i) {
      voltage[i] = driveSubsystem.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble();
      voltageSum =+ voltage[i];
    }
    averageVoltage.set(voltageSum/4);

    return voltage;
  }

  private double[] getModuleVelocity() {
    double[] velocity = new double[4];
    for (int i = 0; i < 4; ++i) {
      velocity[i] = driveSubsystem.getModule(i).getDriveMotor().getVelocity().getValueAsDouble();
    }

    return velocity;
  }

  /**
   * @return robot velocity in meters per second
   */
  private double getRobotVelocity() {
    return driveSubsystem.getKinematics()
        .toChassisSpeeds(driveSubsystem.getState().ModuleStates).vxMetersPerSecond;
  }
}
