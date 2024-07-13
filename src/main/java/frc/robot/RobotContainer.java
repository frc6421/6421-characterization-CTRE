// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.autos.DriveToPositionCommand;
import frc.robot.commands.characterization.StaticFeedforwardCommand;
import frc.robot.commands.characterization.TuneCurrentLimitCommand;
import frc.robot.commands.characterization.TuneVelocityPCommand;
import frc.robot.commands.characterization.VelocityFeedforwardCommand;
import frc.robot.commands.characterization.VerifyOdometryCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.DriveSubsystem.DriveConstants;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Fields \\
  private final DriveSubsystem driveSubsystem;

  private final VerifyOdometryCommand verifyOdometryCommand;
  private final TuneCurrentLimitCommand tuneCurrentLimitCommand;
  private final StaticFeedforwardCommand staticFeedforwardCommand;
  private final VelocityFeedforwardCommand velocityFeedforwardCommand;
  private final TuneVelocityPCommand tuneVelocityPCommand;
  // private final DriveToPositionCommand driveToPositionCommand;

  private final DriveCommand driveCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  
  private final Telemetry telemetry;


  // private SendableChooser<Command> autoChooser;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // Subsystenms \\
    driveSubsystem = new DriveSubsystem();

    // Commands \\
    verifyOdometryCommand = new VerifyOdometryCommand(driveSubsystem);
    tuneCurrentLimitCommand = new TuneCurrentLimitCommand(driveSubsystem);
    staticFeedforwardCommand = new StaticFeedforwardCommand(driveSubsystem);
    velocityFeedforwardCommand = new VelocityFeedforwardCommand(driveSubsystem);
    tuneVelocityPCommand = new TuneVelocityPCommand(driveSubsystem);
    // driveToPositionCommand = new DriveToPositionCommand(driveSubsystem);

    driveCommand = new DriveCommand(driveSubsystem, driverController);
    
    // Defualt Commands \\
    driveSubsystem.setDefaultCommand(driveCommand);

    // AutoChooser \\
    // autoChooser = new SendableChooser<>();

    // autoChooser.setDefaultOption("Tune Position P", driveToPositionCommand);

    // Shuffleboard.getTab("6: Postion P").add("Auto Chooser", autoChooser)
    //   .withPosition(0, 0)
    //   .withSize(2, 1);

    // Telemetry
    telemetry = new Telemetry(DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);
    driveSubsystem.registerTelemetry(telemetry::telemeterize);


    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return null; //autoChooser.getSelected();
  }
}
