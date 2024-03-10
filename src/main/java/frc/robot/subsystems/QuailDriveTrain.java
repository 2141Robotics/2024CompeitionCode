// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.KalmanFilterLocalizer;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.components.GyroModule;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;
import java.util.ArrayList;
import java.util.List;

public class QuailDriveTrain extends SubsystemBase {

  // Quail componenents
  List<QuailSwerveModule> modules = new ArrayList<QuailSwerveModule>();
  private final QuailSwerveDrive driveTrain;

  public SwerveOdometry odometry;
  public PathFollower pathFollower;
  public MiniPID pidcontroller;

  private KalmanFilterLocalizer kalmanFilter =
      new KalmanFilterLocalizer(new Vec2d(0, 0), Constants.LOOPTIME);

  private final GyroModule gyro = new GyroModule();

  private final Field2d field = new Field2d();

  // TODO(Bernie): move to contsnts
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public QuailDriveTrain() {
    // Setup all of our swerve modules
    // TODO: Move to constants
    modules.add(new QuailSwerveModule(new Vec2d(13, 13), 1, 2, 0, 0.443 - 0.25));
    modules.add(new QuailSwerveModule(new Vec2d(13, -13), 3, 4, 1, 0.425 - 0.25));
    modules.add(new QuailSwerveModule(new Vec2d(-13, -13), 5, 6, 2, 0.062 - 0.25));
    modules.add(new QuailSwerveModule(new Vec2d(-13, 13), 7, 8, 3, 0.359 - 0.25));

    // Initialize an underlying Quail drive train + odo
    driveTrain = new QuailSwerveDrive(gyro, modules);
    odometry = new SwerveOdometry(driveTrain);

    Shuffleboard.getTab("DriveTrain").add(this.field);

    // Reset the gyro
    gyro.reset();
  }

  public Pose2d shooterPosition() {
    if (DriverStation.getAlliance().equals(Alliance.Blue)) {
      return new Pose2d();
    } else if (DriverStation.getAlliance().equals(Alliance.Red)) {
      return new Pose2d();
    }
    return new Pose2d();
  }

  public void resetModules() {
    for (QuailSwerveModule module : this.modules) {
      module.reset();
    }
  }

  /**
   * Return the Quail swerve drive object. Deprecated: move to use quaildrivetrain w/ commands
   *
   * @return the Quail swerve drive object
   */
  @Deprecated
  public QuailSwerveDrive getQuailSwerveDrive() {
    return driveTrain;
  }

  /**
   * Retruns the Quail odometry object
   *
   * @return the Quail odometry object
   */
  public SwerveOdometry getOdometry() {
    return odometry;
  }

  /**
   * Get gyro
   *
   * @return the gyro
   */
  public GyroModule getGyro() {
    return gyro;
  }

  /** Stops the drive train (all modules steering + driving) */
  public void stop() {
    for (QuailSwerveModule module : this.modules) {
      module.steeringMotor.stopMotor();
      module.drivingMotor.stopMotor();
    }
  }

  /** Resets the drive train gyro */
  public void resetGyro() {
    this.gyro.reset();
  }

  /////////////////////////////////////////////////////////////////////////////
  // Commands
  /////////////////////////////////////////////////////////////////////////////

  /**
   * Command: Stop the drive train
   *
   * @return the command to stop the drive train
   */
  public Command stopCommand() {
    return this.runOnce(
        () -> {
          this.stop();
        });
  }

  public Command resetModulesCommand() {
    return this.runOnce(
        () -> {
          this.resetModules();
        });
  }

  public Command resetGyroCommand() {
    return this.runOnce(
        () -> {
          this.resetGyro();
        });
  }

  /**
   * Command: Move the drive train TODO: move control logic to here and just have quail do math
   * (This is a mirror of the QuailSwerveDrive.move() method)
   *
   * @param movement the movement to make
   * @param gyroOffset the gyro offset
   * @return the command to move the drive train
   */
  public void move(RobotMovement movement, double gyroOffset) {
    driveTrain.move(movement, gyroOffset);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Telemetry
  /////////////////////////////////////////////////////////////////////////////

  /** Periodically update the odometry for telemetry. */
  @Override
  public void periodic() {
    ArrayList<Vec2d> moduleSpeeds = this.driveTrain.getModuleSpeeds();
    RobotMovement velocity = this.odometry.calculateFastOdometry(moduleSpeeds);

    this.odometry.updateDeltaPoseEstimate(
        velocity.translation.scale(Constants.LOOPTIME).rotate(this.gyro.getAngleDegrees(), true));
    this.odometry.setAngle(-this.gyro.getAngleRadians());

    double[] pos =
        NetworkTableInstance.getDefault()
            .getTable("limelight")
            .getEntry("botpose")
            .getDoubleArray(new double[6]);
    SmartDashboard.putNumberArray("Limelight Pos", pos);
    double LX = 0;
    double LY = 0;
    double LATENCY = 0;

    if (pos.length > 0) {
      LX = pos[1] * Constants.INCHES_PER_METER;
      LY = -pos[0] * Constants.INCHES_PER_METER;
      LATENCY = pos[6];
    }

    SmartDashboard.putNumber("LX", LX);
    SmartDashboard.putNumber("LY", LY);

    double w = 0.15;
    if ((LX == 0) && (LY == 0)) {
      w = 0;
    }

    this.kalmanFilter.update(
        new Vec2d(LX, LY),
        velocity.translation.rotate(-this.gyro.getAngleDegrees(), true),
        LATENCY,
        w,
        Timer.getFPGATimestamp());

    SmartDashboard.putNumber("KFx", this.kalmanFilter.getPose().x);
    SmartDashboard.putNumber("KFy", this.kalmanFilter.getPose().y);

    SmartDashboard.putNumber("Ox", this.odometry.x);
    SmartDashboard.putNumber("Oy", this.odometry.y);

    this.odometry.setPose(
        new Pose2d(this.kalmanFilter.getPose().vec(), this.gyro.getAngleRadians()));

    field.setRobotPose(
        this.odometry.y / Constants.INCHES_PER_METER,
        -this.odometry.x / Constants.INCHES_PER_METER,
        new Rotation2d(this.odometry.theta));
  }

  /** Create the telemetry for the drive train. */
  @Override
  public void initSendable(SendableBuilder builder) {
    for (QuailSwerveModule module : modules) {
      module.initSendable(builder);
    }
  }
}
