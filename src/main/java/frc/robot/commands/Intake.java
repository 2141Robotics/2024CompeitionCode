package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeShooter;

public class Intake extends Command {
  private IntakeShooter intakeShooter;

  public Intake(IntakeShooter intakeShooter) {
    this.intakeShooter = intakeShooter;
    this.addRequirements(this.intakeShooter);
  }

  @Override
  public void execute() {
    this.intakeShooter.stopShooterMotors();
    this.intakeShooter.startIntakeMotors();
  }

  @Override
  public void end(boolean isInterrupted) {
    this.intakeShooter.stopIntakeMotors();
    this.intakeShooter.stopShooterMotors();
  }

  @Override
  public boolean isFinished() {
    return this.intakeShooter.hasNote();
  }
}
