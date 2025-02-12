// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AutoRoutines;
import frc.robot.commands.Intake;
import frc.robot.commands.ManualClimb;
import frc.robot.commands.ManualDrive;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.QuailDriveTrain;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // Initialize all subsystems
  private final QuailDriveTrain s_DriveTrain = new QuailDriveTrain();
  private final IntakeShooter s_IntakeShooter = new IntakeShooter();
  private final Climber s_Climber = new Climber();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  private final CommandXboxController m_SecondaryController =
      new CommandXboxController(Constants.kSecondaryControllerPort);

  // Chooser for autonomous routines
  SendableChooser<Command> m_chooser = new SendableChooser<>();

  // All auto routines
  private final AutoRoutines m_autoRoutines = new AutoRoutines(s_DriveTrain, s_IntakeShooter);

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
    // Shuffleboard.getTab("DriveTrain").add("Gyro", s_DriveTrain.getGyro());

    // Put auto routines on the smart dashboard
    m_chooser.setDefaultOption("shoot preloaded note", this.m_autoRoutines.shootPreloadedNote());
    // m_chooser.addOption("Drive Forward 10 Feet", m_autoRoutines.driveForward10Feet());
    // m_chooser.addOption("Drive to Pose", m_autoRoutines.driveToPose());

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

    s_Climber.setDefaultCommand(new ManualClimb(s_Climber, m_SecondaryController));

    // Bind the reset gyro command to the back button`
    m_driverController
        .back()
        .onTrue(Commands.runOnce(() -> s_DriveTrain.resetGyro(), s_DriveTrain));

    m_driverController.x().whileTrue(this.m_autoRoutines.lineUpShooter());
    m_driverController.a().onTrue(this.s_DriveTrain.xlockModulesCommand());
    m_driverController.b().onTrue(this.s_DriveTrain.setMotorCoastCommand());
    m_driverController.b().onFalse(this.s_DriveTrain.setMotorBrakeCommand());
    // m_driverController.x().whileTrue(new AlignShooter(s_DriveTrain));

    m_SecondaryController.rightBumper().toggleOnTrue(new Intake(s_IntakeShooter));
    m_SecondaryController.back().onTrue(s_Climber.zeroMotorsCommand());
    m_SecondaryController.leftBumper().onTrue(s_IntakeShooter.shoot());
    m_SecondaryController.back().onTrue(s_Climber.zeroMotorsCommand());
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
