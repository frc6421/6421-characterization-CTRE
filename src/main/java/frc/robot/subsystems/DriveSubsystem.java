package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.ClosedLoopOutputType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants.SteerFeedbackType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstantsFactory;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements
 * subsystem so it can be used in command-based projects easily.
 */
public class DriveSubsystem extends SwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier m_simNotifier = null;
  private double m_lastSimTime;
  public ApplyModuleStates autoDriveRequest;

  /* Keep track if we've ever applied the operator perspective before or not */
  private boolean hasAppliedOperatorPerspective = false;

  public static class DriveConstants {
    // Both sets of gains need to be tuned to your individual robot.

    // The steer motor uses any SwerveModule.SteerRequestType control request with
    // the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs STEER_GAINS = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0);
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs DRIVE_GAINS = new Slot0Configs()
        .withKP(0.156).withKI(0).withKD(0)
        .withKS(0.26).withKV(0.14).withKA(0);

    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT = ClosedLoopOutputType.Voltage;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final double SLIP_CURRENT_AMPS = 85; 

    // Theoretical free speed (m/s) at 12v applied output;
    // This needs to be tuned to your individual robot
    public static final double SPEED_AT_12_VOLTS_METERS_PER_SEC = 5.2;

    // Every 1 rotation of the azimuth results in COUPLE_RATIO drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double COUPLE_RATIO = 3.5714285714285716;

    private static final double DRIVE_GEAR_RATIO = 6.12;
    private static final double STEER_GEAR_RATIO = 21.428571428571427;

    // TODO update after initial measurements and before each competition/everytime
    // treads are changed
    private static final double WHEEL_RADIUS_INCHES = 1.99;
    private static final double METERS_PER_INCH = 0.0254;

    private static final double WHEEL_RADIUS_METERS = METERS_PER_INCH * WHEEL_RADIUS_INCHES;

    private static final double WHEEL_CIRCUMFERENCE_METERS = 2 * Math.PI * WHEEL_RADIUS_METERS;

    // Drive Rotations per Meter
    public static final double DRIVE_ROTATIONS_PER_METER = DriveConstants.DRIVE_GEAR_RATIO / WHEEL_CIRCUMFERENCE_METERS;

    private static final boolean STEER_MOTOR_REVERSED = true;
    private static final boolean INVERT_LEFT_SIDE = false;
    private static final boolean INVERT_RIGHT_SIDE = true;

    private static final int PIGEON_CAN_ID = 18;

    // These are only used for simulation
    private static final double STEER_INERTIA = 0.00001;
    private static final double DRIVE_INERTIA = 0.001;
    // Simulated voltage necessary to overcome friction
    private static final double STEER_FRICTION_VOLTAGE = 0.25;
    private static final double DRIVE_FRICTION_VOLTAGE = 0.25;

    private static final SwerveDrivetrainConstants drivetrainConstants = new SwerveDrivetrainConstants()
        .withPigeon2Id(PIGEON_CAN_ID);

    private static final SwerveModuleConstantsFactory constantCreator = new SwerveModuleConstantsFactory()
        .withDriveMotorGearRatio(DRIVE_GEAR_RATIO)
        .withSteerMotorGearRatio(STEER_GEAR_RATIO)
        .withWheelRadius(WHEEL_RADIUS_INCHES)
        .withSlipCurrent(SLIP_CURRENT_AMPS)
        .withSteerMotorGains(STEER_GAINS)
        .withDriveMotorGains(DRIVE_GAINS)
        .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT)
        .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT)
        .withSpeedAt12VoltsMps(SPEED_AT_12_VOLTS_METERS_PER_SEC)
        .withSteerInertia(STEER_INERTIA)
        .withDriveInertia(DRIVE_INERTIA)
        .withSteerFrictionVoltage(STEER_FRICTION_VOLTAGE)
        .withDriveFrictionVoltage(DRIVE_FRICTION_VOLTAGE)
        .withFeedbackSource(SteerFeedbackType.RemoteCANcoder)
        .withCouplingGearRatio(COUPLE_RATIO)
        .withSteerMotorInverted(STEER_MOTOR_REVERSED);

    // Front Left
    private static final int FRONT_LEFT_DRIVE_MOTOR_CAN_ID = 12;
    private static final int FRONT_LEFT_STEER_MOTOR_CAN_ID = 13;
    private static final int FRONT_LEFT_CANCODER_CAN_ID = 13;
    private static final double FRONT_LEFT_ENCODER_OFFSET = -0.28809;

    private static final double FRONT_LEFT_X_POS_INCHES = 8.125;
    private static final double FRONT_LEFT_Y_POS_INCHES = 22.75 / 2;

    // Front Right
    private static final int FRONT_RIGHT_DRIVE_MOTOR_CAN_ID = 10;
    private static final int FRONT_RIGHT_STEER_MOTOR_CAN_ID = 11;
    private static final int FRONT_RIGHT_CANCODER_CAN_ID = 11;
    private static final double FRONT_RIGHT_ENCODER_OFFSET = 0.26978;

    private static final double FRONT_RIGHT_X_POS_INCHES = 8.125;
    private static final double FRONT_RIGHT_Y_POS_INCHES = -22.75 / 2;

    // Back Left
    private static final int BACK_LEFT_DRIVE_MOTOR_CAN_ID = 16;
    private static final int BACK_LEFT_STEER_MOTOR_CAN_ID = 17;
    private static final int BACK_LEFT_CANCODER_CAN_ID = 17;
    private static final double BACK_LEFT_ENCODER_OFFSET = -0.37476;

    private static final double BACK_LEFT_X_POS_INCHES = -12.625;
    private static final double BACK_LEFT_Y_POS_INCHES = 22.75 / 2;

    // Back Right
    private static final int BACK_RIGHT_DRIVE_MOTOR_CAN_ID = 14;
    private static final int BACK_RIGHT_STEER_MOTOR_CAN_ID = 15;
    private static final int BACK_RIGHT_CANCODER_CAN_ID = 15;
    private static final double BACK_RIGHT_ENCODER_OFFSET = 0.02148;

    private static final double BACK_RIGHT_X_POS_INCHES = -12.625;
    private static final double BACK_RIGHT_Y_POS_INCHES = -22.75 / 2;

    private static final SwerveModuleConstants frontLeft = constantCreator.createModuleConstants(
        FRONT_LEFT_STEER_MOTOR_CAN_ID, FRONT_LEFT_DRIVE_MOTOR_CAN_ID, FRONT_LEFT_CANCODER_CAN_ID,
        FRONT_LEFT_ENCODER_OFFSET,
        Units.inchesToMeters(FRONT_LEFT_X_POS_INCHES), Units.inchesToMeters(FRONT_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants frontRight = constantCreator.createModuleConstants(
        FRONT_RIGHT_STEER_MOTOR_CAN_ID, FRONT_RIGHT_DRIVE_MOTOR_CAN_ID, FRONT_RIGHT_CANCODER_CAN_ID,
        FRONT_RIGHT_ENCODER_OFFSET,
        Units.inchesToMeters(FRONT_RIGHT_X_POS_INCHES), Units.inchesToMeters(FRONT_RIGHT_Y_POS_INCHES),
        INVERT_RIGHT_SIDE);
    private static final SwerveModuleConstants backLeft = constantCreator.createModuleConstants(
        BACK_LEFT_STEER_MOTOR_CAN_ID, BACK_LEFT_DRIVE_MOTOR_CAN_ID, BACK_LEFT_CANCODER_CAN_ID, BACK_LEFT_ENCODER_OFFSET,
        Units.inchesToMeters(BACK_LEFT_X_POS_INCHES), Units.inchesToMeters(BACK_LEFT_Y_POS_INCHES), INVERT_LEFT_SIDE);
    private static final SwerveModuleConstants backRight = constantCreator.createModuleConstants(
        BACK_RIGHT_STEER_MOTOR_CAN_ID, BACK_RIGHT_DRIVE_MOTOR_CAN_ID, BACK_RIGHT_CANCODER_CAN_ID,
        BACK_RIGHT_ENCODER_OFFSET,
        Units.inchesToMeters(BACK_RIGHT_X_POS_INCHES), Units.inchesToMeters(BACK_RIGHT_Y_POS_INCHES),
        INVERT_RIGHT_SIDE);

    public static final double DRIVE_SLEW_RATE = 10;

    /** Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d BlueAlliancePerspectiveRotation = Rotation2d.fromDegrees(0);
    /** Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d RedAlliancePerspectiveRotation = Rotation2d.fromDegrees(180);
  }

  /**
   * Custom SwerveRequest that applies SwerveModuleStates for autonomous.
   * Written by 6421 since no standard SwerveRequest that applies SwerveModules
   */
  public class ApplyModuleStates implements SwerveRequest {
    public SwerveModuleState[] States = new SwerveModuleState[] {};

    public SwerveModule.DriveRequestType DriveRequestType = SwerveModule.DriveRequestType.Velocity;

    public SwerveModule.SteerRequestType SteerRequestType = SwerveModule.SteerRequestType.MotionMagic;

    public SwerveRequest.ForwardReference ForwardReference = SwerveRequest.ForwardReference.RedAlliance;

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
      var states = States;
      for (int i = 0; i < modulesToApply.length; ++i) {
        modulesToApply[i].apply(states[i], DriveRequestType, SteerRequestType);
      }

      return StatusCode.OK;
    }

    public ApplyModuleStates withModuleStates(SwerveModuleState[] state) {
      this.States = state;
      return this;
    }

    public ApplyModuleStates withDriveRequestType(SwerveModule.DriveRequestType driveRequestType) {
      this.DriveRequestType = driveRequestType;
      return this;
    }

    public ApplyModuleStates withSteerRequestType(SwerveModule.SteerRequestType steerRequestType) {
      this.SteerRequestType = steerRequestType;
      return this;
    }
  }

  public DriveSubsystem() {
    super(DriveConstants.drivetrainConstants,
        DriveConstants.frontLeft,
        DriveConstants.frontRight,
        DriveConstants.backLeft,
        DriveConstants.backRight);

    autoDriveRequest = new ApplyModuleStates();

    if (Utils.isSimulation()) {
      startSimThread();
    }

  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    m_lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    m_simNotifier = new Notifier(() -> {
      final double currentTime = Utils.getCurrentTimeSeconds();
      double deltaTime = currentTime - m_lastSimTime;
      m_lastSimTime = currentTime;

      /* use the measured time delta, get battery voltage from WPILib */
      updateSimState(deltaTime, RobotController.getBatteryVoltage());
    });
    m_simNotifier.startPeriodic(kSimLoopPeriod);
  }

  /**
   * Used by autonomous SwerveContollerCommand to drive by SwerveModuleState
   * 
   * @param desiredStates from SwerveContollerCommand
   */
  public void autoSetModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.SPEED_AT_12_VOLTS_METERS_PER_SEC);

    autoDriveRequest.withModuleStates(desiredStates);
    autoDriveRequest.apply(m_requestParameters, Modules);
  }

  public Pose2d getPose2d() {
    return getState().Pose;
  }

  @Override
  public void periodic() {
    /* 
     * Periodically try to apply the operator perspective.
     * If we haven't applied the operator perspective before, then we should apply
     * it regardless of DS state.
     * This allows us to correct the perspective in case the robot code restarts
     * mid-match.
     * Otherwise, only check and apply the operator perspective if the DS is
     * disabled.  This ensures driving behavior doesn't change until an explicit 
     * disable event occurs during testing.
     * *** From CTRE Example code.
     */
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance().ifPresent((allianceColor) -> {
        this.setOperatorPerspectiveForward(
            allianceColor == Alliance.Red ? DriveConstants.RedAlliancePerspectiveRotation
                : DriveConstants.BlueAlliancePerspectiveRotation);
        hasAppliedOperatorPerspective = true;
      });
    }
  }

  /**
   * Gets the estimated pose
   * 
   * @return estimated pose from pose estimator (Pose2d)
   */
  public Pose2d getCurrentPose2d() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Set the netrual mode of all swerve modules.
   * 
   * @param neutralMode brake or coast
   * @return OK if all modules neutral mode set successfully or StatusCode of the failed config
   */
  public StatusCode setNeutralMode(NeutralModeValue neutralMode) {
    // Inital value that is not OK
    var status = StatusCode.NotFound;
    // Loop until all modules are set properly or until end of loop
    for (int i = 0; i < 4 ; i++) { 
      status = configNeutralMode(neutralMode);
      // Stop loop if all modules set properly
      if (status.isOK()) {
        return status;
      }
    }
    // Status code if not set properly
    return status;
  }

  // Methods used for characterization \\
  
  /**
   * Get the current module poistion in meters
   * 
   * @param module number of swerve module
   * @return distance in meters
   */
  public double getModuleDistanceMeters(int module) {
    return getModule(module).getDriveMotor().getPosition().getValue() / DriveConstants.DRIVE_ROTATIONS_PER_METER;
  }

  /**
   * Get the kinematics class from CTRE's SwerveDrivetrain class
   * 
   * @return kinematics 
   */
  public SwerveDriveKinematics getSwerveDriveKinematics() {
    return m_kinematics;
  }

}
