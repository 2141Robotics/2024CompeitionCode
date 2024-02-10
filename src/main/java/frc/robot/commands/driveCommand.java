package frc.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.mineinjava.quail.util.geometry.AccelerationLimitedDouble;
import com.mineinjava.quail.util.geometry.AccelerationLimitedVector;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.components.QuailSwerveDrive;


public class driveCommand extends Command{
    public XboxController primaryController;
    public AHRS gyro;
    public QuailSwerveDrive driveTrain;
    public AccelerationLimitedVector a_leftStickVector = new AccelerationLimitedVector(0.1);
    public AccelerationLimitedVector a_rightStickVector = new AccelerationLimitedVector(0.1);

    public AccelerationLimitedVector a_driveVector = new AccelerationLimitedVector(0.003);

    public AccelerationLimitedDouble a_rtrigger = new AccelerationLimitedDouble(0.1);

    public driveCommand(XboxController controller1, AHRS gyro1, QuailSwerveDrive driveTrain1){
        super();
        primaryController = controller1;
        gyro = gyro1;
        driveTrain = driveTrain1;
    }

    @Override
    public void initialize() {
        super.initialize();
        driveTrain.softResetMotors();
        driveTrain.stop();
        System.out.println("drive commmand start");
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
				driveTrain.XLockModules();
			}
		}
		else {
			
			RobotMovement movement = new RobotMovement(rightStickVector.x / 35, newDriveVector);
			driveTrain.move(movement, 0.25 + (this.gyro.getAngle() * Constants.DEG_TO_RAD));
		}
		if(primaryController.getYButton()){
			gyro.reset();
		}
        System.out.println("Drive command done");
    }
    @Override
    public void end(boolean interrupted) {
        System.out.println("drive command end");
        
        super.end(interrupted);
    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
