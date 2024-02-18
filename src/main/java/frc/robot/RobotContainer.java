// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.mineinjava.quail.util.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.ManualDrive;
import frc.robot.commands.RunPath;
import frc.robot.subsystems.QuailDriveTrain;
import java.util.ArrayList;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize all subsystems
  private final QuailDriveTrain s_DriveTrain = new QuailDriveTrain();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  // Chooser for autonomous routines
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // All auto routines
  private final AutoRoutines m_autoRoutines = new AutoRoutines(s_DriveTrain);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    // Add some logging hooks to the command scheduler
    CommandScheduler.getInstance()
        .onCommandInitialize(
            command -> System.out.println("Command initialized: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandInterrupt(
            command -> System.out.println("Command interrupted: " + command.getName()));
    CommandScheduler.getInstance()
        .onCommandFinish(command -> System.out.println("Command finished: " + command.getName()));

    // Put subsystems on the smart dashboard
    Shuffleboard.getTab("DriveTrain").add("Telemetry", s_DriveTrain);
    Shuffleboard.getTab("DriveTrain").add("Gyro", s_DriveTrain.getGyro());

    // Put auto routines on the smart dashboard
    m_chooser.setDefaultOption("Default Auto", m_autoRoutines.defaultAuto());
    m_chooser.addOption("Drive Forward 10 Feet", m_autoRoutines.driveForward10Feet());
    m_chooser.addOption("Drive to Pose", m_autoRoutines.driveToPose());

    Shuffleboard.getTab("Autonomous").add("Select Autonomous Profile", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Make the default command for the drive train the drive command
    s_DriveTrain.setDefaultCommand(new ManualDrive(m_driverController, s_DriveTrain));

    // Bind the reset gyro command to the back button`
    m_driverController
        .back()
        .onTrue(Commands.runOnce(() -> s_DriveTrain.resetGyro(), s_DriveTrain));

    // Bind the reset modules command to the start button
    m_driverController
        .start()
        .onTrue(Commands.runOnce(() -> s_DriveTrain.resetModulesCommand(), s_DriveTrain));

    ArrayList<Pose2d> zero = new ArrayList<Pose2d>();
    zero.add(new Pose2d(0, 0, 0));
    m_driverController.x().whileTrue(new RunPath(s_DriveTrain, zero));
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
