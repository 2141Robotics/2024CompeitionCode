package frc.robot.components;

import com.kauailabs.navx.frc.AHRS;

/**
 * Simple wrapper around the AHRS gryo class that makes it easier to use and
 * pass around
 *
 * <p>
 * The goal is to make it easier to use the gyro in other classes and
 * standardize the way we use
 * it
 */
public class GyroModule {

  private final AHRS gyro;

  public GyroModule() {
    this.gyro = new AHRS();
  }

  /**
   * Get the gyro object (deprecated, use getAngleDegrees() or getAngleRadians()
   * instead
   *
   * @return the gyro object
   */
  @Deprecated
  public AHRS getGyro() {
    return gyro;
  }

  /**
   * Returns the angle in degrees
   *
   * @return the angle in degrees (0 to 360)
   */
  public double getAngleDegrees() {
    return gyro.getAngle();
  }

  /**
   * Returns the angle in radians
   *
   * @return the angle in radians (0 to 2pi)
   */
  public double getAngleRadians() {
    return Math.toRadians(gyro.getAngle());
  }

  /** Resets the gyro to 0 degrees */
  public void reset() {
    System.out.println("Resetting gyro...");
    gyro.reset();
  }

  /**
   * Returns whether or not the gyro is currently calibrating
   *
   * @return true if the gyro is calibrating, false otherwise
   */
  public boolean isCalibrating() {
    return gyro.isCalibrating();
  }
}
