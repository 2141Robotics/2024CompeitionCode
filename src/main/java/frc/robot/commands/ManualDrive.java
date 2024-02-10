package frc.robot.commands;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.mineinjava.quail.util.geometry.AccelerationLimitedDouble;
import com.mineinjava.quail.util.geometry.AccelerationLimitedVector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.components.GyroModule;
import frc.robot.subsystems.QuailDriveTrain;


public class ManualDrive extends Command{
    private final XboxController primaryController;

    private final GyroModule gyro;
    private final QuailDriveTrain driveTrain;

    private final AccelerationLimitedVector a_leftStickVector = new AccelerationLimitedVector(0.1);
    private final AccelerationLimitedVector a_rightStickVector = new AccelerationLimitedVector(0.1);
    private final AccelerationLimitedVector a_driveVector = new AccelerationLimitedVector(0.003);
    private final AccelerationLimitedDouble a_rtrigger = new AccelerationLimitedDouble(0.1);

    public ManualDrive(XboxController controller1, GyroModule gyro, QuailDriveTrain driveTrain){
        super();
        primaryController = controller1;
        this.gyro = gyro;
        this.driveTrain = driveTrain;

        addRequirements(driveTrain);
    }

    @Override
    public void initialize() {
        super.initialize();
        driveTrain.stop();
        System.out.println("Starting drive command...");
    }

    @Override
    public void execute() {
        super.execute();
        double leftX = primaryController.getLeftX();
		double leftY = - primaryController.getLeftY(); /// Y UP is negative
		double rightY = -primaryController.getRightY();
		double rightX = -primaryController.getRightX();

		double rightTrigger = primaryController.getRightTriggerAxis();
		
		Vec2d leftStickVector = new Vec2d(leftX, leftY);
        Vec2d rightStickVector = new Vec2d(rightX, rightY);

        Vec2d lstick = a_leftStickVector.update(leftStickVector);
        Vec2d rstick = a_rightStickVector.update(rightStickVector);
        double a_rtriggerValue = a_rtrigger.update(rightTrigger);

		double speedScale = 0.08 + (0.92 * rightTrigger);


		if (Math.abs(rstick.x) < 0.1){
			rightX = Double.MIN_NORMAL;
		}

		if (lstick.getLength() < Constants.deadZonePercent) {
			leftStickVector = new Vec2d(0,0);
		}
        if (rstick.getLength() < Constants.deadZonePercent) {
			rightStickVector = new Vec2d(0,0);
		}
        Vec2d driveVector = leftStickVector.normalize().scale(speedScale);
        Vec2d newDriveVector = a_driveVector.update(driveVector);


		if ((Math.abs(rstick.x) < 0.1) && (lstick.getLength() < 0.05)){
			driveTrain.stop();
			if (primaryController.getAButton()) {
				driveTrain.getQuailSwerveDrive().XLockModules();
			}
		}
		else {
			
			RobotMovement movement = new RobotMovement(rightStickVector.x / 35, newDriveVector);
			driveTrain.move(movement, 0.25 + (this.gyro.getAngleRadians()));
		}
		if(primaryController.getYButton()){
			gyro.reset();
		}
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Simple drive command ending");
        super.end(interrupted);
    }
}
