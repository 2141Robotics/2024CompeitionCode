// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;

import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.geometry.Pose2d;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private RobotContainer m_robotContainer;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    m_robotContainer = new RobotContainer();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();

    // ArrayList<Vec2d> moduleSpeeds = drivetrain.getModuleSpeeds();
    // RobotMovement robotMovement = odometry.calculateFastOdometry(moduleSpeeds);

    // Vec2d deltaTranslation = robotMovement.translation.scale(0.02).rotate(this.gyro.getAngle(), true);

    // this.odometry.updateDeltaPoseEstimate(deltaTranslation);

    // odometry.setAngle(-gyro.getAngle() * Constants.DEG_TO_RAD);
    
    // this.odometry.setAngle(this.gyro.getAngle());

    // SmartDashboard.putNumber("x", this.odometry.x);
    // SmartDashboard.putNumber("y", this.odometry.y);

    // if (controller.getBButtonPressed()){
		// 	odometry.setPose(new Pose2d(0,0,0));
		// }
    // if (controller.getYButtonPressed()){
    //   this.gyro.reset();
    // }

		// double[] pos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
		// SmartDashboard.putNumberArray("Limelight Pos", pos);
		// SmartDashboard.putNumber("LX", -pos[1] * Constants.METERS_TO_INCHES);
		// SmartDashboard.putNumber("LY", pos[0] * Constants.METERS_TO_INCHES);

		// if(pos[0] != 0 && pos[1] != 0){
		// 	odometry.setPose(new Pose2d(-pos[1]* Constants.METERS_TO_INCHES, pos[0]* Constants.METERS_TO_INCHES, -this.gyro.getAngle() * Constants.DEG_TO_RAD));
		// }
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {

  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // Get the autonomous command from the container
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

		// driveCommand dt = new driveCommand(primaryController, gyro, drivetrain);
		// CommandScheduler.getInstance().schedule(dt);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    // double velo = controller.getLeftTriggerAxis();

    // Vec2d dv = new Vec2d(this.controller.getLeftX(), -this.controller.getLeftY());

    // VelocityVoltage command = new VelocityVoltage(100 * velo);
    // talon1.setControl(command); // motors are configured to run in opposite
    // directions. Make both negative to invert
    // talon2.setControl(command);

    /*
     * if (controller.getAButton()) {
     * talon1.set(-velo);
     * talon2.set(-v elo);
     * }
     * else {
     * talon1.set(velo);
     * talon2.set(velo);
     * }
     */
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {

  }
}
