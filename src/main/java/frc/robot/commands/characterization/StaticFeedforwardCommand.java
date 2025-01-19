// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;


import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class StaticFeedforwardCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private final ChassisSpeeds velocityDelta;
  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.PointWheelsAt zeroWheelRequest;
  private final SwerveRequest.ApplyRobotSpeeds stopRobotRequest;
  private final SwerveRequest.ApplyRobotSpeeds driveByRobotSpeedsRequest;

  /* What to publish over networktables for static feedforward */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Static feedforward table publishers */
  private final NetworkTable staticFeedForwardTable = inst.getTable("3 StaticFeedforward");

  private final DoubleArrayPublisher moduleVoltage = staticFeedForwardTable.getDoubleArrayTopic("Module Voltage").publish();

  /**
   * Distance used to determine if the robot is moving.
   * </p>
   * *** May want to change value depending on data output.
   */
  private final static double ROBOT_IS_MOVING_METERS = 0.001;

  /**
   * Used to determine the feedforward voltage needed to just get the robot to
   * move.
   * </p>
   * When done update the kS value of the Drive Motor
   * {@link TunerConstants#DRIVE_GAINS}.
   * </p>
   * *** May need to be updated if the weight of the robot changes significantly.
   * 
   * @param driveSubsystem the drive subsystem used with command.
   */
  public StaticFeedforwardCommand(CommandSwerveDrivetrain driveSubsystem) {
    this.driveSubsystem = driveSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);

    // Initialize
    setChassisSpeeds = new ChassisSpeeds();

    // The change in velocity to add each robot cycle.
    velocityDelta = new ChassisSpeeds(0.0001, 0, 0);

    // Swerve request to make sure the wheels point in the x direction
    zeroWheelRequest = new PointWheelsAt();

    // Swerve request to stop the robot
    stopRobotRequest = new ApplyRobotSpeeds();
    
    // Swerve request to use to drive the robot
    driveByRobotSpeedsRequest = new ApplyRobotSpeeds();

    SmartDashboard.putData("Static Feedforward Command", this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // Set wheels to point forward (x direction)
    // Can add "".withModuleDirection(Rotation2D)" to turn all modules to a specific
    // angle if other directions want to be tested
    driveSubsystem.setControl(zeroWheelRequest);
    driveSubsystem.seedFieldCentric();
    driveSubsystem.tareEverything();

    // Set chassis speed to zero each time the command starts
    setChassisSpeeds = new ChassisSpeeds();

    // Set moduleVoltage array values with intitial voltage (should be zero)
    moduleVoltage.set(getModuleVoltage());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Drive forward at set Chassis Speed
    driveSubsystem.setControl(driveByRobotSpeedsRequest.withSpeeds(setChassisSpeeds));

    // The current voltage of each drive motor module.
    moduleVoltage.set(getModuleVoltage());

    // Increase chassis speed.
    setChassisSpeeds = setChassisSpeeds.plus(velocityDelta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // The final voltage of each drive motor module.
    moduleVoltage.set(getModuleVoltage());

    // Stop the robot.
    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Return true if robot moves 1 cm
    return Math.abs(driveSubsystem.getState().Pose.getX()) >= ROBOT_IS_MOVING_METERS;
  }

  private double[] getModuleVoltage() {
    double[] voltage = new double[4];
    for (int i = 0; i < 4; ++i) {
      voltage[i] = driveSubsystem.getModule(i).getDriveMotor().getMotorVoltage().getValueAsDouble();
    }
    return voltage;
  }
}
