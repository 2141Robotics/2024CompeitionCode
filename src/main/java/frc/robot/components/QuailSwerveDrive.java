package frc.robot.components;

import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.util.geometry.Vec2d;
import java.util.ArrayList;
import java.util.List;

/**
 * Swerve drive instance
 *
 * @author 2141 Spartonics
 */
public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule> {
  /** A timer used to continually reset the motors. Prevents overshooting the rotation. */
  private static int resetTimer = 0;

  /** The gryoscope used for rotation measurements. */
  private final GyroModule gyroModule;

  /** A list of all of the swerve modules on the drivetrain. */
  private final List<QuailSwerveModule> modules;

  /**
   * @param minSpeed minimum movement speed (0 to 1)
   * @param maxSpeed maximum movement speed (0 to 1)
   * @param maxRotation maximum rotational speed (0 to 1)
   * @param gyroscope the swerve drive's gyroscope
   * @param QuailSwerveModules the swerve drive's wheel modules
   */
  public QuailSwerveDrive(GyroModule gyroModule, List<QuailSwerveModule> modules) {
    super(modules);
    this.modules = modules;
    this.gyroModule = gyroModule;
  }

  /**
   * Checks if the robot is resetting or if the gyro is callibrating.
   *
   * @return Whether or not the robot can drive
   */
  public boolean canDrive() {
    return resetTimer <= 0 && !this.gyroModule.isCalibrating();
  }

  @Override
  public String toString() {
    // The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
    StringBuilder builder = new StringBuilder("SwerveDrive[");

    for (int i = 0; i < this.modules.size(); i++) {
      builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
    }

    builder.delete(builder.length() - 2, builder.length());
    builder.append("]");
    return builder.toString();
  }

  public void setBrake() {
    this.modules.forEach(
        (QuailSwerveModule m) -> {
          m.setDriveMotorBrake();
        });
  }

  public void setCoast() {
    this.modules.forEach(
        (QuailSwerveModule m) -> {
          m.setDriveMotorCoast();
        });
  }

  /**
   * Fetches the current speeds of the swerve modules.
   *
   * @return An array of the current speeds of the swerve modules
   */
  public ArrayList<Vec2d> getModuleSpeeds() {
    ArrayList<Vec2d> vectors = new ArrayList<Vec2d>();
    for (QuailSwerveModule module : this.swerveModules) {
      vectors.add(((QuailSwerveModule) module).getCurrentMovement());
    }
    return vectors;
  }
}
