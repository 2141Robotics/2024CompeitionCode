package frc.robot.components;

import frc.robot.math.Constants;

import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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
	public final double analogEncoderOffset;
	
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
		System.out.println("|||||||||" + analogEncoderID + "|||||||||");
		this.analogEncoderOffset = analogEncoderOffset;
		this.analogEncoderID = analogEncoderID;
	}
	
	/**
	 * Configures the motors and sets the steering motor's rotation to zero.
	 */
	public void init()
	{
		this.steeringMotor.restoreFactoryDefaults();
		this.pidController = this.steeringMotor.getPIDController();

		kP = 0.4;
		kI = 0.01;
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

		this.steeringMotor.burnFlash();

    	System.out.println("done config");

		// Reset the motor rotations.
		this.reset();
	}

	// returns rotations, 0 is x axis
	public double getAbsoluteEncoderAngle() {
		double currentPos = this.analogEncoder.getAbsolutePosition() - this.analogEncoderOffset;

		currentPos = (currentPos + 1) % 1;

		return currentPos;
	}

	/**
	 * Sets steering motor's rotation to zero.
	 */
	public void reset()
	{
		System.out.println("RESET");
		
		this.steeringMotor.getEncoder().setPosition(getAbsoluteEncoderAngle() * Constants.steeringRatio);
		this.currentAngle = (getAbsoluteEncoderAngle() * Constants.TWO_PI);
		this.setAngle(0);
		
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

	public Vec2d getCurrentMovement() {
		return new Vec2d();
	}

	public AnalogEncoder getEncoder() {
		return this.analogEncoder;
	}

	public void putEncoderDash() {
		SmartDashboard.putNumber("Encoder value " + this.analogEncoderID, this.getAbsoluteEncoderAngle());
		SmartDashboard.putNumber("Motor value " + this.analogEncoderID, this.steeringMotor.getEncoder().getPosition() / 12.8);
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
