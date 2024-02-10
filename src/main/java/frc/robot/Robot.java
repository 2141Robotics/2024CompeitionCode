// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.RunPath;
import frc.robot.commands.driveCommand;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;
import frc.robot.math.Constants;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.ConstraintsPair;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AnalogEncoder;


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

  public QuailSwerveDrive drivetrain;

  public SwerveOdometry odometry;

  public PathFollower pathFollower;

  public MiniPID pidcontroller;

  public XboxController primaryController = new XboxController(0);

  public double E1 = 0.944;
  public double E2 = 0.928;
  public double E3 = 0.566;
  public double E4 = 0.859;

  AHRS gyro = new AHRS();

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public XboxController controller = new XboxController(0);

  // public TalonFX talon1;
  // public TalonFX talon2;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.\

    m_robotContainer = new RobotContainer();
    List<QuailSwerveModule> modules = new ArrayList<QuailSwerveModule>();

    /**
     * Encoder Offsets (value of encoder when physical wheel position = 0) in rotations
     */

    modules.add(new QuailSwerveModule(new Vec2d(-13, -13), 1, 2, 0,E1-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(-13, 13), 3, 4, 1, E2-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(13, 13), 5, 6, 2, E3-0.25));
    modules.add(new QuailSwerveModule(new Vec2d(13, -13), 7, 8, 3, E4-0.25));

    drivetrain = new QuailSwerveDrive(gyro, modules);

    odometry = new SwerveOdometry(drivetrain);

    // talon1 = new TalonFX(41);
    // talon2 = new TalonFX(42);

    var talonFXConfigs = new TalonFXConfiguration();

    // talon1.getConfigurator().apply(talonFXConfigs);
    // talon2.getConfigurator().apply(talonFXConfigs);

    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.0;
    slot0Configs.kA = 0.00;
    slot0Configs.kV = 0;
    slot0Configs.kP = 0.5;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0;

    // talon1.getConfigurator().apply(slot0Configs);
    // talon2.getConfigurator().apply(slot0Configs);

    // talon1.setInverted(true);
    // talon2.setInverted(false);

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

    ArrayList<Vec2d> moduleSpeeds = drivetrain.getModuleSpeeds();
    RobotMovement robotMovement = odometry.calculateFastOdometry(moduleSpeeds);

    Vec2d deltaTranslation = robotMovement.translation.scale(0.02).rotate(this.gyro.getAngle(), true);

    this.odometry.updateDeltaPoseEstimate(deltaTranslation);

    odometry.setAngle(-gyro.getAngle() * Constants.DEG_TO_RAD);
    
    this.odometry.setAngle(this.gyro.getAngle());

    SmartDashboard.putNumber("x", this.odometry.x);
    SmartDashboard.putNumber("y", this.odometry.y);

    if (controller.getBButtonPressed()){
			odometry.setPose(new Pose2d(0,0,0));
		}
    if (controller.getYButtonPressed()){
      this.gyro.reset();
    }

		double[] pos = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose").getDoubleArray(new double[6]);
		SmartDashboard.putNumberArray("Limelight Pos", pos);
		SmartDashboard.putNumber("LX", -pos[1] * Constants.METERS_TO_INCHES);
		SmartDashboard.putNumber("LY", pos[0] * Constants.METERS_TO_INCHES);

		if(pos[0] != 0 && pos[1] != 0){
			odometry.setPose(new Pose2d(-pos[1]* Constants.METERS_TO_INCHES, pos[0]* Constants.METERS_TO_INCHES, -this.gyro.getAngle() * Constants.DEG_TO_RAD));
		}

    // Add a button to trigger calibration
    // TODO(Bernie): Confirm this works, might need to be a subclass of WPILIB command
    // SmartDashboard.putData("Calibrate absolute encoders", new drivetrain.calibrateAbosoluteEncoders());
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
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    /*
    points.add(new Pose2d(-20,80,0));
    points.add(new Pose2d(-20, 180,0));
    points.add(new Pose2d(-20, 80,0));
    */
    points.add(new Pose2d(0,0,0));
    points.add(new Pose2d(0, 20,0));
    points.add(new Pose2d(0, 0,0));


    this.pidcontroller = new MiniPID(0.0, 0.000, 0);
    this.pidcontroller.setF(0.0);

    ConstraintsPair tp = new ConstraintsPair(200, 300);
    ConstraintsPair rp = new ConstraintsPair(0.1, 1);
    this.pathFollower = new PathFollower(odometry, null, tp, rp, pidcontroller, 3, 4, 1,15);

    m_autonomousCommand = new RunPath(pathFollower, drivetrain, points, odometry);

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

		driveCommand dt = new driveCommand(primaryController, gyro, drivetrain);
		CommandScheduler.getInstance().schedule(dt);
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
    double velo = controller.getLeftTriggerAxis();

    Vec2d dv = new Vec2d(this.controller.getLeftX(), -this.controller.getLeftY());

    VelocityVoltage command = new VelocityVoltage(100 * velo);
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
