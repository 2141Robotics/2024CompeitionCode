package frc.robot;

/**
 * Container for various static variables.
 * 
 * @author 2141 Spartonics
 */
public class Constants
{

	/** 2π */
	public static final double TWO_PI = 2d * Math.PI;

	/** π/2 */
	public static final double PI_OVER_TWO = Math.PI / 2d;

	/** Multiply a radian value by this to convert it to degrees. */
	public static final double RAD_TO_DEG = 180d / Math.PI;

	public static final double DEG_TO_RAD = 1/ RAD_TO_DEG;

	/** Angle precision for rotation */
	public static final double ANGLE_PRECISION = Math.PI / 32;

	public static final int INCHES_PER_FOOT = 12;

	public static final double steeringRatio = 12.8d;
	public static final double driveRatio = 6.12d;

	public static final double deadZonePercent = 0.05d;

	public static final double METERS_TO_INCHES = 39.3701;

	public static final int kDriverControllerPort = 0;
	public static final double wheelDiameter = 3.95d;
}
