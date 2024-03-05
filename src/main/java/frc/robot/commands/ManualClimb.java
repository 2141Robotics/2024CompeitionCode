package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ManualClimb extends Command {
  private Climber climber;
  private CommandXboxController secondaryController;

  public ManualClimb(Climber climber, CommandXboxController secondaryController) {
    this.climber = climber;
    this.secondaryController = secondaryController;
  }

  public void initialize() {
    climber.stopMotors();
  }

  public void execute() {
    double x = this.secondaryController.getRightX();
    double y = -this.secondaryController.getRightY();

    if (Math.abs(x) < 0.3) { // use a larger deadzone because x won't be used as much
      x = 0;
    }

    if (Math.abs(y) < Constants.deadZonePercent) {
      y = 0;
    }

    double rightMotorSpeed = y;
    rightMotorSpeed += (x * Constants.CLIMBER_DIFF_SPEED);
    double leftMotorSpeed = y;
    leftMotorSpeed -= (x * Constants.CLIMBER_DIFF_SPEED);

    this.climber.setMotorSpeeds(
        leftMotorSpeed * Constants.CLIMBER_SPEED, rightMotorSpeed * Constants.CLIMBER_SPEED);
  }

  public void end() {
    System.out.println("climber command end");
    this.climber.stopMotors();
  }
}
