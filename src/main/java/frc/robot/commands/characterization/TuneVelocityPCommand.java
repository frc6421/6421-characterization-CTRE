// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.swerve.SwerveRequest.ApplyRobotSpeeds;
import com.ctre.phoenix6.swerve.SwerveRequest.PointWheelsAt;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TuneVelocityPCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private Slot0Configs slot0Configs;

  private ChassisSpeeds setChassisSpeeds;

  private final PointWheelsAt zeroWheelsRequest;
  private final ApplyRobotSpeeds driveVelocityRequest;
  private final ApplyRobotSpeeds stopRobotRequest;

  private double velocity;
  private double velocityP;
  private double finalRobotVelocity;

  /**
   * TODO UPDATE COMMENT
   * TODO ADD ERROR CONTROL
   */
  public TuneVelocityPCommand(CommandSwerveDrivetrain drive) {
    driveSubsystem = drive;

    slot0Configs = new Slot0Configs();

    zeroWheelsRequest = new PointWheelsAt();
    driveVelocityRequest = new ApplyRobotSpeeds();
    stopRobotRequest = new ApplyRobotSpeeds();

    velocity = 0;
    velocityP = 0;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drive);

    Shuffleboard.getTab("5: Velocity P").add(this);
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
        slot0Configs.withKP(velocityP);
        // apply the config to the j-th module and verify that it returns a OK StatusCode
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
    driveSubsystem.seedFieldCentric();;
    driveSubsystem.tareEverything();
    // Set wheels to face forward.
    driveSubsystem.setControl(zeroWheelsRequest);
    // Set chassis speed based on the velocity set in Shuffleboard
    setChassisSpeeds = new ChassisSpeeds(velocity, 0, 0);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setControl(driveVelocityRequest.withSpeeds(setChassisSpeeds));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

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
   * The p value to set for the velocity PID.
   * 
   * @param velocityP in units of voltage / rpm (?)
   */
  public void setVelocityP(double velocityP) {
    this.velocityP = velocityP;
  }

  /**
   * Set velocity to run the robot at
   * 
   * @param velocity x velocity of the robot (-
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC} to
   *                 {@link DriveSubsystem#SPEED_AT_12_VOLTS_METERS_PER_SEC})
   */
  public void setVelocity(double velocity) {
    this.velocity = MathUtil.clamp(velocity, -1 * TunerConstants.kSpeedAt12Volts.magnitude(),
    TunerConstants.kSpeedAt12Volts.magnitude());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty("Velocity", () -> velocity, this::setVelocity);
    builder.addDoubleProperty("Velocity P", () -> velocityP, this::setVelocityP);
    builder.addDoubleProperty("Module 0 Velocity", () -> getModuleVelocity(0), null);
    builder.addDoubleProperty("Module 1 Velocity", () -> getModuleVelocity(1), null);
    builder.addDoubleProperty("Module 2 Velocity", () -> getModuleVelocity(2), null);
    builder.addDoubleProperty("Module 3 Velocity", () -> getModuleVelocity(3), null);
    //builder.addDoubleProperty("Robot Velocity", () -> getRobotVelocity(), null);
    builder.addDoubleProperty("Final Robot Velocity", () -> finalRobotVelocity, null);

  }

  // TODO verify module units: rpm or m/s?
  private double getModuleVelocity(int module) {
    return driveSubsystem.getModule(module).getDriveMotor().getVelocity().getValueAsDouble();
  }

  /**
   * @return robot velocity in meters per second
   */
  private double getRobotVelocity() {
    return driveSubsystem.getState().Speeds.vxMetersPerSecond;
  }

}
