// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.RunPath;
import frc.robot.components.GyroModule;
import frc.robot.subsystems.QuailDriveTrain;
import frc.robot.subsystems.ServoController;

import java.util.ArrayList;
import java.util.Collection;

import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize the gyro module
  private final GyroModule m_gyro = new GyroModule();

  // Initialize all subsystems
  private final QuailDriveTrain s_DriveTrain = new QuailDriveTrain(m_gyro);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(Constants.kDriverControllerPort);

  // Chooser for autonomous routines
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // All auto routines
  private final AutoRoutines m_autoRoutines = new AutoRoutines(s_DriveTrain, m_gyro);

  private final ServoController servoController = new ServoController();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Put subsystems w/ their active commands on the smart dashboard
    Shuffleboard.getTab("DriveTrain").add("Telemetry", s_DriveTrain);

    Shuffleboard.getTab("DriveTrain").add(m_gyro.getGyro());

    // Put subsystem state on the smart dashboard

    // Put manual buttons on the smart dashboard
    Shuffleboard.getTab("DriveTrain").add("Zero Absolute Encoders", s_DriveTrain.resetAbsoluteEncoders());

    // Put auto routines on the smart dashboard
    m_chooser.setDefaultOption("Default Auto", m_autoRoutines.defaultAuto());
    m_chooser.addOption("Drive Forward 10 Feet", m_autoRoutines.driveForward10Feet());
    m_chooser.addOption("Drive to Pose", m_autoRoutines.driveToPose());
    m_chooser.addOption("Drive forward 100 feet", m_autoRoutines.driveForward100Feet());

    Shuffleboard.getTab("Autonomous").add("Select Autonomous Profile", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    // .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is
    // pressed,
    // cancelling on release.
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    // Make the default command for the drive train the drive command
    m_driverController.back().onTrue(Commands.runOnce(() -> {
              m_gyro.reset();}));

    m_driverController.a().onTrue(this.servoController.setEnableShooting());
    m_driverController.b().onTrue(this.servoController.setDisableShooting());

    m_driverController.y().onTrue(this.s_DriveTrain.resetOdometry());

    m_driverController.start().onTrue(s_DriveTrain.resetModulesCommand());
    s_DriveTrain.setDefaultCommand(new ManualDrive(m_driverController, m_gyro, s_DriveTrain));
    ArrayList<Pose2d> zero = new ArrayList<Pose2d>();
    zero.add(new Pose2d(0,0,0));
    m_driverController.x().whileTrue(new RunPath(s_DriveTrain,zero));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return m_chooser.getSelected();
  }
}
