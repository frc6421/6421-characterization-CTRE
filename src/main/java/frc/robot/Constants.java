// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.*;
import static edu.wpi.first.units.Units.*;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static class AutoConstants {

    public static final double THETA_P = 5; // TODO update values
    public static final double THETA_I = 0; // TODO update values
    public static final double THETA_D = 0; // TODO update values

    public static final double X_DRIVE_P = 2.31; // TODO update values
    public static final double X_DRIVE_I = 0;
    public static final double X_DRIVE_D = 0;

    public static final double Y_DRIVE_P = 2.31; // TODO update values
    public static final double Y_DRIVE_I = 0;
    public static final double Y_DRIVE_D = 0;

    public static final LinearVelocity AUTO_MAX_VELOCITY_METERS_PER_SECOND = MetersPerSecond.of(4.0); // TODO update
                                                                                                      // value
    public static final LinearAcceleration AUTO_MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = MetersPerSecondPerSecond
        .of(5.0); // TODO update value

    public static final AngularVelocity AUTO_MAX_ANGULAR_VELOCITY_RAD_PER_SEC = RadiansPerSecond.of(2 * Math.PI);
    public static final AngularAcceleration AUTO_MAX_ANGULAR_ACCELERATION_RAD_PER_SEC = RadiansPerSecondPerSecond
        .of(2 * Math.PI);
  }

  public static class DriveConstants {
    public static final double DRIVE_SLEW_RATE = 10.0;
  }

}
