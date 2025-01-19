// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;



import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class TuneCurrentLimitCommand extends Command {

  private final CommandSwerveDrivetrain driveSubsystem;

  private final ChassisSpeeds velocityDelta;
  private ChassisSpeeds setChassisSpeeds;

  private final SwerveRequest.PointWheelsAt zeroWheelRequest;
  private final SwerveRequest.ApplyRobotSpeeds stopRobotRequest;
  private final SwerveRequest.ApplyRobotSpeeds driveByChassisSpeedsRequest;

    /* What to publish over networktables for static feedforward */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Static feedforward table publishers */
  private final NetworkTable currentLimitTable = inst.getTable("2 CurrentLimit");

  private final DoubleArrayPublisher statorCurrentPub = currentLimitTable.getDoubleArrayTopic("Stator Current").publish();
  private final DoubleArrayPublisher supplyCurrentPub = currentLimitTable.getDoubleArrayTopic("Supply Current").publish();

  
  // Constants \\
  // TODO determine the velocity to set which means the wheels are slipping.
  /** In meters per second */
  private static final double VELOCITY_LIMIT = 0.1;

  /**
   * Used to find the current at which wheels start slipping.
   * </p>
   * To use place robot against wall or other unmovable barrier and start command.
   * </p>
   * When done update the value in {@link TunerConstants#kSlipCurrent}
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
    zeroWheelRequest = new SwerveRequest.PointWheelsAt();
    // Swerve request to stop the robot
    stopRobotRequest = new SwerveRequest.ApplyRobotSpeeds();
    // Swerve request to use to drive the robot
    driveByChassisSpeedsRequest = new SwerveRequest.ApplyRobotSpeeds();


    // Add command to SmartDashboard.
    SmartDashboard.putData("Current Limit Command", this);
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
    statorCurrentPub.set(getmoduleStatorCurrent());
    supplyCurrentPub.set(getModuleSupplyCurrent());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Start driving forward
    driveSubsystem.setControl(driveByChassisSpeedsRequest.withSpeeds(setChassisSpeeds));

    // Get the current stator and supply currents for each module's drive motor
    statorCurrentPub.set(getmoduleStatorCurrent());
    supplyCurrentPub.set(getModuleSupplyCurrent());

    // Increament the X velocity.
    setChassisSpeeds = setChassisSpeeds.plus(velocityDelta);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Get the final stator and supply currents for each module's drive motor
    statorCurrentPub.set(getmoduleStatorCurrent());
    supplyCurrentPub.set(getModuleSupplyCurrent());

    // Stop the motors.
    driveSubsystem.setControl(stopRobotRequest);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Stop command if any wheel is slipping
    return areWheelsSlipping();
  }

  /**
   * Determine if any wheels is slipping. A wheel is slipping if the velocity is
   * higher than {@link TuneCurrentLimitCommand#VELOCITY_LIMIT}.
   * 
   * @return true if any wheel is slipping, otherwise false
   */
  private boolean areWheelsSlipping() {
    // Cycle through all module drive motors to see if any wheels is slipping.
    for (int i = 0; i < 4; ++i) {
      if (driveSubsystem.getModule(i).getCurrentState().speedMetersPerSecond >= VELOCITY_LIMIT) {
        return true;
      }
    }
    return false;
  }

  private double[] getmoduleStatorCurrent() {
    double[] statorCurrent = new double[4];
    for (int i = 0; i < 4; ++i) {
      statorCurrent[i] = driveSubsystem.getModule(i).getDriveMotor().getStatorCurrent().getValueAsDouble();
    }

    return statorCurrent;
  }

  private double[] getModuleSupplyCurrent() {
    double[] supplyCurrent = new double[4];
    for (int i = 0; i < 4; ++i) {
      supplyCurrent[i] = driveSubsystem.getModule(i).getDriveMotor().getSupplyCurrent().getValueAsDouble();
    }

    return supplyCurrent;
  }
}
