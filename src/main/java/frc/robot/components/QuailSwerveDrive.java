package frc.robot.components;

import java.util.ArrayList;
import java.util.List;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.kauailabs.navx.frc.AHRS;
import com.mineinjava.quail.SwerveDrive;
import com.mineinjava.quail.util.geometry.Vec2d;

import frc.robot.math.Constants;


/**
 * Container for the robot's swerve drivetrain. Wraps a gyroscope and swerve modules.
 * 
 * @author 2141 Spartonics
 */
public class QuailSwerveDrive extends SwerveDrive<QuailSwerveModule>
{
	// TODO: move me to a better home :)
	public static final double ABSOLUTE_ENCODER_OFFSET_POLL_EVENTS = 50; // How many times to poll the absolute encoder for calibration

	/** A timer used to continually reset the motors. Prevents overshooting the rotation. */
	private static int resetTimer = 0;
	/** The gryoscope used for rotaiton measurements. */
	private final AHRS gyro;
	/** A list of all of the swerve modules on the drivetrain. */
	private final List<QuailSwerveModule> modules;
	
	/**
	 * @param minSpeed minimum movement speed (0 to 1)
	 * @param maxSpeed maximum movement speed (0 to 1)
	 * @param maxRotation maximum rotational speed (0 to 1)
	 * @param gyroscope the swerve drive's gyroscope
	 * @param QuailSwerveModules the swerve drive's wheel modules
	 */
	public QuailSwerveDrive(AHRS gyroscope, List<QuailSwerveModule> modules)
	{
		super(modules);
		this.gyro = gyroscope;
		this.modules = modules;
		for (QuailSwerveModule module : modules) {
			module.init();
		}
	}
	public void softResetMotors() {
		this.modules.forEach(m -> m.init());
	}

	
	/**
	 * Set a timer for reseting the motors.
	 * Used to reset the motors multiple times.
	 */
	public void resetMotors()
	{
		this.modules.forEach(m -> m.reset());

	}

	/**
	 * Reset the gyro to 0°.
	 */
	public void resetGyro()
	{
		this.gyro.reset();
	}

	/**
	 * Calibrate the absolute encoders offsets. 
	 * 
	 * Note! This should only be called with all the steering motors set to face forward.
	 */
	public void calibrateAbosoluteEncoders() {
		System.out.println("Calibrating absolute encoders");

		// Calculate the average of the absolute encoder values for each module for the duration of the calibration
		ArrayList<Double> sums = new ArrayList<Double>();
		for (int i = 0; i < this.modules.size(); i++) {
			sums.add(0.0);
		}

		// Get a sum of the absolute encoder values for each module for n poll events
		for (int i = 0; i < ABSOLUTE_ENCODER_OFFSET_POLL_EVENTS; i++) {
			for (int j = 0; j < this.modules.size(); j++) {
				sums.set(j, sums.get(j) + this.modules.get(j).getRawAbsoluteEncoderAngle());
			}
		}

		// Set the offset of each module to the average value
		for (QuailSwerveModule module : this.modules) {
			module.updateAbsoluteEncoderOffset(sums.get(this.modules.indexOf(module)) / ABSOLUTE_ENCODER_OFFSET_POLL_EVENTS);
		}

		System.out.println("Absolute encoder offset calibration complete!");
	}
	

	/**
	 * Checks if the robot is resetting or if the gyro is callibrating.
	 * 
	 * @return Whether or not the robot can drive
	 */
	public boolean canDrive()
	{
		return resetTimer <= 0 && !this.gyro.isCalibrating();
	}

	/**
	 * Getter for the gyroscope.
	 * 
	 * @return The gyroscope.
	 */
	public AHRS getGyro()
	{
		return this.gyro;
	}

	/**
	 * Getter for the swerve modules.
	 * 
	 * @return The swerve modules.
	 */
	public List<QuailSwerveModule> getModules()
	{
		return this.modules;
	}

	@Override
	public String toString()
	{
		// The class will be represented as "SwerveDrive[Module1 = {}, Module2 = {}, ...]"
		StringBuilder builder = new StringBuilder("SwerveDrive[");

		for(int i = 0; i < this.modules.size(); i++)
		{
			builder.append("Module" + i + " = {" + this.modules.get(i) + "}, ");
		}

		builder.delete(builder.length() - 2, builder.length());
		builder.append("]");
		return builder.toString();
	}

	public double gettingangle(){
		return this.gyro.getAngle() * Constants.DEG_TO_RAD;
	}


	public void stop(){
		for(QuailSwerveModule module : this.modules) {
			module.drivingMotor.set(0);
		}
	}

    /** 
	 * Set modules into coast mode 
	*/

	public ArrayList<Vec2d> getModuleSpeeds() {
        ArrayList<Vec2d> vectors = new ArrayList<Vec2d>();
        for (QuailSwerveModule module : this.swerveModules) {
            vectors.add(((QuailSwerveModule) module).getCurrentMovement());
        }
        return vectors;
    }
}
