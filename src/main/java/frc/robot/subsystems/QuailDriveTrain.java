// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.Constants;

import java.util.ArrayList;
import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.GyroModule;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;

public class QuailDriveTrain extends SubsystemBase {

  // Quail componenents
  List<QuailSwerveModule> modules = new ArrayList<QuailSwerveModule>();
  private final QuailSwerveDrive driveTrain;

  public SwerveOdometry odometry;
  public PathFollower pathFollower;
  public MiniPID pidcontroller;

  private final GyroModule gyro;

  private final Field2d field = new Field2d();

  // TODO(Bernie): move to contsnts
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  // Telemetry data
  public ArrayList<Double> absoluteEncoderValues = new ArrayList<Double>();
  public ArrayList<Double> motorEncoderValues = new ArrayList<Double>();
  public ArrayList<Double> encoderRawValues = new ArrayList<Double>();

  public QuailDriveTrain(GyroModule gyro) {
    // Setup all of our swerve modules
    // TODO: Move to constants
    modules.add(new QuailSwerveModule(new Vec2d(13, 13), 1, 2, 0, 0.443-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(13, -13), 3, 4, 1, 0.425-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(-13, -13), 5, 6, 2, 0.062-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(-13, 13), 7, 8, 3, 0.359-0.25));

    // Initialize an underlying Quail drive train + odo
    driveTrain = new QuailSwerveDrive(gyro, modules);
    odometry = new SwerveOdometry(driveTrain);

    this.gyro = gyro;

    // Initialize arrays for tracking position, velocity, and acceleration
    for (int i = 0; i < modules.size(); i++) {
      absoluteEncoderValues.add(0.0);
      motorEncoderValues.add(0.0);
      encoderRawValues.add(0.0);
    }
    Shuffleboard.getTab("DriveTrain").add(this.field);
  }

  public void resetModules() {
    for (QuailSwerveModule module : this.modules) {
      module.reset();
    }
  }

  public Command resetModulesCommand(){
    return this.runOnce(() -> {this.resetModules();});
  }

  /**
   * Return the Quail swerve drive object.
   * Deprecated: move to use quaildrivetrain w/ commands
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

  public void stop() {
    for (QuailSwerveModule module : this.modules) {
      module.drivingMotor.set(0);
    }
  }

  /////////////////////////////////////////////////////////////////////////////
  // Commands
  /////////////////////////////////////////////////////////////////////////////

  /**
   * Command: Reset the absolute encoders for all swerve modules
   * 
   * @return the command to reset the absolute encoders
   */
  public Command resetAbsoluteEncoders() {
    return this.runOnce(() -> this.driveTrain.calibrateAbosoluteEncoders());
  }

  /**
   * Command: Stop the drive train
   * 
   * @return the command to stop the drive train
   */
  public Command stopCommand() {
    return this.runOnce(() -> {
      this.stop();
    });
  }

  /**
   * Command: Move the drive train
   * TODO: move control logic to here and just have quail do math
   * (This is a mirror of the QuailSwerveDrive.move() method)
   * 
   * @param movement   the movement to make
   * @param gyroOffset the gyro offset
   * @return the command to move the drive train
   */
  public void move(RobotMovement movement, double gyroOffset) {
    // System.out.println("Moving drive train..." + movement.toString() + " " +
    // gyroOffset);
    driveTrain.move(movement, gyroOffset);
  }

  /////////////////////////////////////////////////////////////////////////////
  // Telemetry
  /////////////////////////////////////////////////////////////////////////////

  /**
   * Periodically update the odometry for telemetry.
   */
  @Override
  public void periodic() {
    // Update our telemetry data
    for (int i = 0; i < modules.size(); i++) {
      absoluteEncoderValues.set(i, modules.get(i).getAbsoluteEncoderAngle());
      motorEncoderValues.set(i, modules.get(i).steeringMotor.getEncoder().getPosition() / Constants.GEAR_RATIO_SWERVE);
      encoderRawValues.set(i, modules.get(i).getRawAbsoluteEncoderAngle());
    }
    ArrayList<Vec2d> moduleSpeeds = this.driveTrain.getModuleSpeeds();
    RobotMovement velocity = this.odometry.calculateFastOdometry(moduleSpeeds);

    this.odometry.updateDeltaPoseEstimate(velocity.translation.scale(Constants.LOOPTIME).rotate(this.gyro.getAngleDegrees(), true));
    this.odometry.setAngle(-this.gyro.getAngleRadians());

    field.setRobotPose(this.odometry.y / Constants.INCHES_PER_METER, -this.odometry.x / Constants.INCHES_PER_METER, new Rotation2d(this.odometry.theta));
  }

  public boolean test() {
    return true;
  }

  /**
   * Create the telemetry for the drive train.
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    // Add telemetry data for each module in the drive train
    // This is a little hacky because of some java constraints
    builder.addDoubleProperty("Absolute Encoder 0", () -> absoluteEncoderValues.get(0), null);
    builder.addDoubleProperty("Absolute Encoder 1", () -> absoluteEncoderValues.get(1), null);
    builder.addDoubleProperty("Absolute Encoder 2", () -> absoluteEncoderValues.get(2), null);
    builder.addDoubleProperty("Absolute Encoder 3", () -> absoluteEncoderValues.get(3), null);

    builder.addDoubleProperty("Motor Encoder 0", () -> motorEncoderValues.get(0), null);
    builder.addDoubleProperty("Motor Encoder 1", () -> motorEncoderValues.get(1), null);
    builder.addDoubleProperty("Motor Encoder 2", () -> motorEncoderValues.get(2), null);
    builder.addDoubleProperty("Motor Encoder 3", () -> motorEncoderValues.get(3), null);


    builder.addDoubleProperty("raw Encoder 0", () -> encoderRawValues.get(0), null);
    builder.addDoubleProperty("raw Encoder 1", () -> encoderRawValues.get(1), null);
    builder.addDoubleProperty("raw Encoder 2", () -> encoderRawValues.get(2), null);
    builder.addDoubleProperty("raw Encoder 3", () -> encoderRawValues.get(3), null);
  }
}
