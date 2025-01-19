// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.characterization;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class VerifyOdometryCommand extends Command {
  private final CommandSwerveDrivetrain driveSubsystem;

  private final SwerveRequest.PointWheelsAt zeroWheels;

  private boolean hasStatusCodeError;

  /* What to publish over networktables for telemetry */
  private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

  /* Verify odometry table publishers */
  private final NetworkTable verifyOdometryTable = inst.getTable("1 VerifyOdometry");

  private final DoublePublisher robotMeters = verifyOdometryTable.getDoubleTopic("Robot Meters").publish();
  private final DoublePublisher module0Meters = verifyOdometryTable.getDoubleTopic("Module 0 Meters").publish();
  private final DoublePublisher module1Meters = verifyOdometryTable.getDoubleTopic("Module 1 Meters").publish();
  private final DoublePublisher module2Meters = verifyOdometryTable.getDoubleTopic("Module 2 Meters").publish();
  private final DoublePublisher module3Meters = verifyOdometryTable.getDoubleTopic("Module 3 Meters").publish();
  private final StringEntry odometryMessage = verifyOdometryTable.getStringTopic("Messages").getEntry("NaN");

  /**
   * Verify that odometry calcualtion is same as actual distance travelled.
   * </p>
   * If odometry is off by more than a acceptable range for the team, it is
   * suggested that the wheel be physically measured with a caliper and the
   * measured value entered into {@link generated.TunerConstants#kWheelRadius}.
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

    SmartDashboard.putData("Verify Odometry Command", this);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    odometryMessage.set("Init");
    // Point wheels forward (x direction)
    // can add "".withModuleDirection(Rotation2D)" to turn all modules to a specific
    // angle if other directions want to be tested
    driveSubsystem.applyRequest(()-> zeroWheels);

    // Make current rotation forward.
    driveSubsystem.seedFieldCentric();
    // Reset odometry to 0,0
    driveSubsystem.tareEverything();

    // Set drive motors to coast. Check to see if successful
    hasStatusCodeError = (driveSubsystem.configNeutralMode(NeutralModeValue.Coast) == StatusCode.OK) ? false : true;
    if (hasStatusCodeError) {
      odometryMessage.set("*** Set Neutral Mode FAILED initialize() ***");
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
 @Override
  public void execute() {

    // Publish data to tables
    if (!odometryMessage.get().equals("Execute")){
      odometryMessage.set("Execute");
    }
    robotMeters.set(driveSubsystem.getStateCopy().Pose.getX());
    module0Meters.set(driveSubsystem.getModule(0).getPosition(true).distanceMeters);
    module1Meters.set(driveSubsystem.getModule(1).getPosition(true).distanceMeters);
    module2Meters.set(driveSubsystem.getModule(2).getPosition(true).distanceMeters);
    module3Meters.set(driveSubsystem.getModule(3).getPosition(true).distanceMeters);

    // Do nothing. Manually move the robot to distances.
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    odometryMessage.set("Final robot distance (meters) :" + String.valueOf(driveSubsystem.getState().Pose.getX()));

    // Set drive motors to brake. Check to see if successful
    hasStatusCodeError = (driveSubsystem.configNeutralMode(NeutralModeValue.Brake) == StatusCode.OK) ? false : true;
    if (hasStatusCodeError) {
      odometryMessage.set("*** Set Neutral Mode FAILED end() ***");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return hasStatusCodeError;
  }
}