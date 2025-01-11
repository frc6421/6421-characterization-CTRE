// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.autos.DriveToPositionCommand;
import frc.robot.commands.characterization.StaticFeedforwardCommand;
import frc.robot.commands.characterization.TuneCurrentLimitCommand;
import frc.robot.commands.characterization.TuneVelocityPCommand;
import frc.robot.commands.characterization.VelocityFeedforwardCommand;
import frc.robot.commands.characterization.VerifyOdometryCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class RobotContainer {
  private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
  // 3/4 of a rotation per second max angular velocity
  private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); 

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private final CommandXboxController joystick = new CommandXboxController(0);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  /* Commands */
  private final VerifyOdometryCommand verifyOdometryCommand;
  private final TuneCurrentLimitCommand tuneCurrentLimitCommand;
  private final StaticFeedforwardCommand staticFeedforwardCommand;
  private final VelocityFeedforwardCommand velocityFeedforwardCommand;
  private final TuneVelocityPCommand tuneVelocityPCommand;

  private final DriveToPositionCommand driveToPositionCommand;

  private final SendableChooser<Command> autoChooser;
  private final SlewRateLimiter xDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);
  private final SlewRateLimiter yDriveSlew = new SlewRateLimiter(Constants.DriveConstants.DRIVE_SLEW_RATE);

  public RobotContainer() {
    DriverStation.silenceJoystickConnectionWarning(true);
    
    // Commands \\
    verifyOdometryCommand = new VerifyOdometryCommand(drivetrain);
    tuneCurrentLimitCommand = new TuneCurrentLimitCommand(drivetrain);
    staticFeedforwardCommand = new StaticFeedforwardCommand(drivetrain);
    velocityFeedforwardCommand = new VelocityFeedforwardCommand(drivetrain);
    tuneVelocityPCommand = new TuneVelocityPCommand(drivetrain);
    driveToPositionCommand = new DriveToPositionCommand(drivetrain);

    // AutoChooser \\
    autoChooser = new SendableChooser<>();

    autoChooser.setDefaultOption("Tune Position P", driveToPositionCommand);

    Shuffleboard.getTab("6: Position P").add(autoChooser);

    configureBindings();
  }

  private void configureBindings() {
    // Note that X is defined as forward according to WPILib convention,
    // and Y is defined as to the left according to WPILib convention.
    drivetrain.setDefaultCommand(
        // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive
          // Drive forward with negative Y (forward)
          .withVelocityX(xDriveSlew.calculate(-joystick.getLeftY() * MaxSpeed)) 
          // Drive left with negative X (left)
          .withVelocityY(yDriveSlew.calculate(-joystick.getLeftX() * MaxSpeed))
          // Drive counterclockwise with negative X (left) 
          .withRotationalRate(-joystick.getRightX() * MaxAngularRate) 
        ));

    joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
    joystick.b().whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))));

    // Run SysId routines when holding back/start and X/Y.
    // Note that each routine should be run exactly once in a single log.
    joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
    joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
    joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
    joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    // reset the field-centric heading on left bumper press
    joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
