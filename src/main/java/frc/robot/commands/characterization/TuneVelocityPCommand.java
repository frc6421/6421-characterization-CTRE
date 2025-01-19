// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TuneVelocityPCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private Slot0Configs slot0Configs;

  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.ApplyRobotSpeeds driveVelocityRequest;
  private final SwerveRequest.ApplyRobotSpeeds stopRobotRequest;

  /* What to publish over networktables for static feedforward */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Static feedforward table publishers */
  private final NetworkTable velocityPTable = inst.getTable("5 Velocity P");

  private final DoublePublisher finalVelocityPub = velocityPTable.getDoubleTopic("Final Velocity").publish();

  /**
   * TODO UPDATE COMMENT
   * TODO ADD ERROR CONTROL
   */
  public TuneVelocityPCommand(CommandSwerveDrivetrain drive) {
    driveSubsystem = drive;

    slot0Configs = new Slot0Configs();

    driveVelocityRequest = new SwerveRequest.ApplyRobotSpeeds();
    stopRobotRequest = new SwerveRequest.ApplyRobotSpeeds();

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    SmartDashboard.putData("Velocity P Command", this);
    SmartDashboard.putNumber("5: Velocity P", 0);
    SmartDashboard.putNumber("5: Velocity", 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // loop multiple times to make sure that the config was applied properly
    for (int i = 0; i < 4; ++i) {
      var isError = false;
      // apply P value to each module
      for (int j = 0; j < 4; ++j) {
        // get the current config from j-th module
        driveSubsystem.getModule(j).getDriveMotor().getConfigurator().refresh(slot0Configs);
        // update the P value of the config
        slot0Configs.withKP(SmartDashboard.getNumber("5: Velocity P:", 0));
        // apply the config to the j-th module and verify that it returns a OK
        // StatusCode
        if (isError == driveSubsystem.getModule(j).getDriveMotor().getConfigurator().apply(slot0Configs).isError()) {
          // break the for loop if the config apply didn't work on any module.
          System.out.println("Module " + j + "config FAILED");
          break;
        }
      }
      // break the for loop if config apply worked.
      if (!isError) {
        System.out.println("Apply config successful");
        break;
      } else {
        System.out.println("Apply config NOT successful");
      }
    }

    // Reset odometry
    driveSubsystem.seedFieldCentric();
    driveSubsystem.tareEverything();

    // Set chassis speed based on the velocity set in SmartDashboard. Check to make
    // sure within max velocity of robot.
    double xVelocity = MathUtil.clamp(SmartDashboard.getNumber("5: Velocity", 0),
        -1 * TunerConstants.kSpeedAt12Volts.magnitude(),
        TunerConstants.kSpeedAt12Volts.magnitude());
    setChassisSpeeds = new ChassisSpeeds(xVelocity, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    finalVelocityPub.set(getRobotVelocity());

    driveSubsystem.setControl(stopRobotRequest);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop the command when the robot has travelled at least 3.5 meters.
    return Math.abs(driveSubsystem.getState().Pose.getX()) > 3.5;
  }

  /**
   * @return robot velocity in meters per second
   */
  private double getRobotVelocity() {
    return driveSubsystem.getState().Speeds.vxMetersPerSecond;
  }

}
