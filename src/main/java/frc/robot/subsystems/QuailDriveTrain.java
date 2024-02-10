// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.components.GyroModule;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;

public class QuailDriveTrain extends SubsystemBase {

  // Quail componenents
  List<QuailSwerveModule> modules;
  private final QuailSwerveDrive driveTrain;

  public SwerveOdometry odometry;
  public PathFollower pathFollower;
  public MiniPID pidcontroller;

  // TODO(Bernie): move to contsnts
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  public QuailDriveTrain(GyroModule gyro) {
    // Setup all of our swerve modules
    // TODO: Move to constants
    modules.add(new QuailSwerveModule(new Vec2d(-13, -13), 1, 2, 0, 0));
    modules.add(new QuailSwerveModule(new Vec2d(-13, 13), 3, 4, 1, 0));
    modules.add(new QuailSwerveModule(new Vec2d(13, 13), 5, 6, 2, 0));
    modules.add(new QuailSwerveModule(new Vec2d(13, -13), 7, 8, 3, 0));

    // Initialize an underlying Quail drive train + odo
    driveTrain = new QuailSwerveDrive(gyro, modules);
    odometry = new SwerveOdometry(driveTrain);
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a
   * digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
