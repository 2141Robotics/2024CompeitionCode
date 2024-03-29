package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.AccelerationLimitedVector;
import com.mineinjava.quail.util.geometry.Vec2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.QuailDriveTrain;

public class ManualDrive extends Command {
  private final CommandXboxController primaryController;

  private final QuailDriveTrain driveTrain;

  private final AccelerationLimitedVector a_driveVector =
      new AccelerationLimitedVector(1000); // 0.003);

  public ManualDrive(CommandXboxController controller1, QuailDriveTrain driveTrain) {
    super();
    primaryController = controller1;
    this.driveTrain = driveTrain;

    addRequirements(driveTrain);
  }

  @Override
  public void initialize() {
    super.initialize();
    // driveTrain.stop();
    driveTrain.resetModules();
  }

  @Override
  public void execute() {
    super.execute();
    double leftX = primaryController.getLeftX();
    double leftY = -primaryController.getLeftY(); // / Y UP is negative
    double rightY = -primaryController.getRightY();
    double rightX = primaryController.getRightX();

    double rightTrigger = primaryController.getRightTriggerAxis();

    Vec2d leftStickVector = new Vec2d(leftX, leftY);
    Vec2d rightStickVector = new Vec2d(rightX, rightY);

    double speedScale = 0.08 + (0.92 * rightTrigger);

    // UNKOWN USE
    // if (Math.abs(leftStickVector.x) < 0.1) {
    //  rightX = Double.MIN_NORMAL;
    // }

    if (leftStickVector.getLength() < Constants.deadZonePercent) {
      leftStickVector = new Vec2d(0, 0);
    }
    if (rightStickVector.getLength() < Constants.deadZonePercent) {
      rightStickVector = new Vec2d(0, 0);
    }
    Vec2d driveVector = leftStickVector.normalize().scale(speedScale);
    Vec2d newDriveVector = a_driveVector.update(driveVector);

    if ((Math.abs(rightStickVector.x) < 0.1) && (newDriveVector.getLength() < 0.05)) {
      driveTrain.stop();
    } else {
      RobotMovement movement = new RobotMovement(-rightStickVector.x / 35, newDriveVector);

      // TODO(bernie): remove me - used for testing
      // movement = new RobotMovement(0, new Vec2d(0, 0.05));

      driveTrain.move(movement, (driveTrain.getGyro().getAngleRadians()));
    }
  }

  @Override
  public void end(boolean interrupted) {
    System.out.println("Simple drive command ending");
    super.end(interrupted);
  }
}
