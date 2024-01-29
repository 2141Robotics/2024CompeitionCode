// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.driveCommand;
import frc.robot.components.QuailSwerveDrive;
import frc.robot.components.QuailSwerveModule;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;

  public QuailSwerveDrive drivetrain;

  public SwerveOdometry odometry;

	public PathFollower pathFollower;

	public MiniPID pidcontroller;
  public RelativeEncoder encoder1;
  public RelativeEncoder encoder2;
  public RelativeEncoder encoder3;
  public RelativeEncoder encoder4;

  AHRS gyro = new AHRS();

  
	private static final XboxController PRIMARY_CONTROLLER = new XboxController(0);

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */


  CANSparkMax motor1;
  CANSparkMax motor2;
  CANSparkMax motor3;
  CANSparkMax motor4;
  CANSparkMax motor5;
  CANSparkMax motor6;
  CANSparkMax motor7;
  CANSparkMax motor8;

  SparkPIDController pidController1;
  SparkPIDController pidController2;
  SparkPIDController pidController3;
  SparkPIDController pidController4;
  SparkPIDController pidController5;
  SparkPIDController pidController6;
  SparkPIDController pidController7;
  SparkPIDController pidController8;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  public int rotations = 1;
  public XboxController controller = new XboxController(0);

  public TalonFX talon1;
  public TalonFX talon2;

  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.\
    


    m_robotContainer = new RobotContainer();
    List<QuailSwerveModule> modules = new ArrayList<QuailSwerveModule>();

		modules.add(new QuailSwerveModule(new Vec2d(-13, 13), 1, 2, 11, 0));
		modules.add(new QuailSwerveModule(new Vec2d(-13, -13), 3, 4, 12, 0));
		modules.add(new QuailSwerveModule(new Vec2d(13,-13), 5, 6, 13, 0));
		modules.add(new QuailSwerveModule(new Vec2d(13,13), 7, 8, 14, 0));

		drivetrain = new QuailSwerveDrive(gyro, modules);

		odometry = new SwerveOdometry(drivetrain);

    motor1 = new CANSparkMax(1, MotorType.kBrushless);
    motor2 = new CANSparkMax(2, MotorType.kBrushless);
    motor3 = new CANSparkMax(3, MotorType.kBrushless);
    motor4 = new CANSparkMax(4, MotorType.kBrushless);
    motor5 = new CANSparkMax(5, MotorType.kBrushless);
    motor6 = new CANSparkMax(6, MotorType.kBrushless);
    motor7 = new CANSparkMax(7, MotorType.kBrushless);
    motor8 = new CANSparkMax(8, MotorType.kBrushless);

    pidController1 = motor1.getPIDController();
    pidController2 = motor2.getPIDController();
    pidController3 = motor3.getPIDController();
    pidController4 = motor4.getPIDController();
    pidController5 = motor5.getPIDController();
    pidController6 = motor6.getPIDController();
    pidController7 = motor7.getPIDController();
    pidController8 = motor8.getPIDController();

    
    encoder1 = motor2.getEncoder();
    encoder2 = motor4.getEncoder();
    encoder3 = motor6.getEncoder();
    encoder4 = motor8.getEncoder();


    talon1 = new TalonFX(41);
    talon2 = new TalonFX(42);

    kP = 0.5; 
    kI = 0.000;
    kD = 0; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = 1; 
    kMinOutput = -1;

    // set PID coefficients
    pidController1.setP(kP);
    pidController1.setI(kI);
    pidController1.setD(kD);
    pidController1.setIZone(kIz);
    pidController1.setFF(kFF);
    pidController1.setOutputRange(kMinOutput, kMaxOutput);

    var talonFXConfigs = new TalonFXConfiguration();

    talon1.getConfigurator().apply(talonFXConfigs);
    talon2.getConfigurator().apply(talonFXConfigs);

    var slot0Configs = new Slot0Configs();
		slot0Configs.kS = 0.0; 
		slot0Configs.kA = 0.00; 
		slot0Configs.kV = 0;
		slot0Configs.kP = 0.5;
		slot0Configs.kI = 0;
		slot0Configs.kD = 0;

    talon1.getConfigurator().apply(slot0Configs);
    talon2.getConfigurator().apply(slot0Configs);

    talon1.setInverted(true);
    talon2.setInverted(false);


  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.

    driveCommand dt = new driveCommand(PRIMARY_CONTROLLER, gyro, drivetrain);
		CommandScheduler.getInstance().schedule(dt);


    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {

    double velo = controller.getLeftTriggerAxis();

    // EARL LOOK HERE
    VelocityVoltage command = new VelocityVoltage(100 * velo);
    //talon1.setControl(command); // motors are configured to run in opposite directions. Make both negative to invert
    //talon2.setControl(command);

    if (controller.getAButton()) {
      talon1.set(-velo);
      talon2.set(-velo);
    }
    else {
      talon1.set(velo);
      talon2.set(velo);
    }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
    Vec2d vec = new Vec2d(1, 1);
    setRawAngle(vec);
    setRawVelocity(vec);
  }  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

  public void setRawAngle(Vec2d vec){
    double angle = vec.getAngle();
    pidController1.setReference(0.25+(angle/(2*Math.PI)) * 12.8, CANSparkMax.ControlType.kPosition);
  }
  public void setRawVelocity(Vec2d vec){
    double velo = vec.getLength();
    pidController1.setReference(velo * 12.8, CANSparkMax.ControlType.kPosition);
  }
}
