package frc.robot;

/**
 * Container for various static variables.
 *
 * @author 2141 Spartonics
 */
public class Constants {
  // general constants
  public static final double LOOPTIME = 0.02; // / looptime in seconds

  // angle constants
  /** 2π */
  public static final double TWO_PI = 2d * Math.PI;

  /** π/2 */
  public static final double PI_OVER_TWO = Math.PI / 2d;

  // unit conversions
  public static final double RAD_TO_DEG = 180d / Math.PI;
  public static final double DEG_TO_RAD = 1 / RAD_TO_DEG;
  public static final double INCHES_PER_METER = 39.3701;
  public static final int INCHES_PER_FOOT = 12;

  // precision constants
  public static final double deadZonePercent = 0.05d;

  /** Angle precision for rotation */
  public static final double ANGLE_PRECISION = Math.PI / 32;

  // gear ratios
  public static final double steeringRatio = 12.8d;
  public static final double driveRatio = 6.12d;
  public static final double GEAR_RATIO_SWERVE = 12.8d; // TODO: merge w steeringratio

  // controller ports
  public static final int kDriverControllerPort = 0;
  public static final int kSecondaryControllerPort = 1;

  // drivetrain constants
  public static final double wheelDiameter = 3.95d;

  // climber constants
  public static final double CLIMBER_SPEED = 0.2d; // speed for climber
  public static final double CLIMBER_DIFF_SPEED = 0.4; // x speed for climber
}
