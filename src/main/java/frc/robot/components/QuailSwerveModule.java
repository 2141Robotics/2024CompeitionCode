package frc.robot.components;

import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

/**
 * Container for one swerve module. Wraps two falcon500s: one for driving and one for steering.
 * 
 * @author 2141 Spartonics
 */
public class QuailSwerveModule extends SwerveModuleBase
{
	/** The PID id used to determine what PID settings to use. */
	/** The motor controlling the module's movement. */
	public final CANSparkMax drivingMotor;
	/** The motor controlling the module's rotation. */
	public final CANSparkMax steeringMotor;
	/** The can coder measuring the module's absolute rotaiton. */
	private final AnalogEncoder analogEncoder;
	/** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
	
	public SparkPIDController pidController;

	public final double analogEncoderID;
	public double analogEncoderOffset;
	
	public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;


	/**
	 * @param driveMotor driving motor ID
	 * @param steeringMotor steering motor ID
	 * @param canCoder can coder ID
	 * @param rotationDirection the steering motor's rotational direction, usually perpendicular to the center of the robot
	 * @param canCoderOffset the can coder's rotational offset
	 */

	public QuailSwerveModule(Vec2d position, int driveMotorID, int steeringMotorID, int analogEncoderID, double analogEncoderOffset)
	{
		super(position, Constants.steeringRatio, Constants.driveRatio, true);
		this.drivingMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		this.steeringMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
		this.analogEncoder = new AnalogEncoder(analogEncoderID);
		this.analogEncoderOffset = analogEncoderOffset;
		this.analogEncoderID = analogEncoderID;
		this.steeringMotor.restoreFactoryDefaults();
		this.pidController = this.steeringMotor.getPIDController();

		kP = 0.2;
		kI = 0.00;
		kD = 0.00;
		kIz = 0;
		kFF = 0.000;
		kMaxOutput = 1;
		kMinOutput = -1;

		pidController.setP(kP);
    	pidController.setI(kI);
    	pidController.setD(kD);
    	pidController.setIZone(kIz);
    	pidController.setFF(kFF);
    	pidController.setOutputRange(kMinOutput, kMaxOutput);
		this.steeringMotor.setInverted(false);
		this.steeringMotor.setIdleMode(IdleMode.kBrake);
		this.drivingMotor.setIdleMode(IdleMode.kBrake);
		this.drivingMotor.burnFlash();
		this.steeringMotor.burnFlash();
		System.out.println("Finished initializing" + this.toString());
	}
	
	/**
	 * Sets the module's angle to the desired angle.
	 * TODO: Bernie thinks this is a no-op
	 */
	@Deprecated
	public void init()
	{
		// Reset the motor rotations.
		System.out.println("RESET");
		// this.steeringMotor.getEncoder().setPosition(getAbsoluteEncoderAngle() * Constants.steeringRatio);
		// this.currentAngle = (getAbsoluteEncoderAngle() * Constants.TWO_PI);
		// this.setAngle(this.currentAngle);
		
		/* 
		// Set the steering motor's internal rotation to 0.
		double currentPos = this.canCoder.getAbsolutePosition().refresh().getValue();
		// The angle to rotate to face forward.
		double angleToRotate = currentPos > 0.5d ? currentPos - 1d : currentPos;
		// Set the steering motor's rotation.
		this.steeringMotor.setPosition(angleToRotate * 12.8);
		this.currentAngle = angleToRotate * Constants.TWO_PI;
		*/
	}

	// returns rotations, 0 is x axis
	public double getAbsoluteEncoderAngle() {
		double currentPos = this.analogEncoder.getAbsolutePosition() - this.analogEncoderOffset;
		currentPos = (currentPos + 1) % 1;
		return currentPos;
	}

	/**
	 * Returns the steering motor's angle without any modifications.
	 * 
	 * @return The steering motor's rotation in rotations (-1 to 1)
	 */
	public double getRawAbsoluteEncoderAngle() {
		return this.analogEncoder.getAbsolutePosition();
	}

	/**
	 * Updates the steering motor's rotational offset.
	 * Note! This should be done with the robot on the ground and the wheels facing forward as straight as possible.
	 * 
	 * @param newAbsoluteEncoderOffset The new rotational offset in rotations (-1 to 1)
	 */
	public void updateAbsoluteEncoderOffset(double newAbsoluteEncoderOffset) {
		if (newAbsoluteEncoderOffset > 1 || newAbsoluteEncoderOffset < -1) {
			throw new IllegalArgumentException("The new absolute encoder offset must be between -1 and 1");
		}

		System.out.println("Updating encoder offset for analogEncoderId: " + this.analogEncoderID + " to " + this.analogEncoder.getAbsolutePosition() + " from " + this.analogEncoderOffset);
		this.analogEncoderOffset = newAbsoluteEncoderOffset;
	}

	public Vec2d getCurrentMovement() {
		double speed = this.drivingMotor.getEncoder().getVelocity();
		speed = speed / 60; // convert to seconds
		speed = speed / Constants.driveRatio; // convert to real wheel rotations
		speed = speed * Constants.wheelDiameter * Math.PI;
		return new Vec2d(this.getAbsoluteEncoderAngle() * Constants.TWO_PI, speed, false);
	}

	@Override
	public void setRawAngle(double angle)
	{

		if (angle != angle) {
			return;
		}

		this.steeringMotor.getPIDController().setReference((angle/(2*Math.PI)) * 12.8, CANSparkMax.ControlType.kPosition);
	}

	@Override
	public void setRawSpeed(double speed) {
		this.drivingMotor.set(speed);
	}

	/**
	 * Getter for the drive motor.
	 * 
	 * @return The drive motor
	 */
     	public CANSparkMax getDriveMotor()
	{
		return this.drivingMotor;
	}

	/**
	 * Getter for the steering motor.
	 * 
	 * @return The steering motor.
	 */
	public CANSparkMax getSteeringMotor()
	{
		return this.steeringMotor;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving Motor ID = ?, Cancoder ID = ?]"
		return "SwerveModule[Steering Motor ID = " + this.steeringMotor.getDeviceId() + ", Driving Motor ID = " + this.drivingMotor.getDeviceId() + ", Cancoder ID = " + this.analogEncoderID + "]";
	}

	
}
