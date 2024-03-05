package frc.robot.components;

import com.mineinjava.quail.SwerveModuleBase;
import com.mineinjava.quail.util.geometry.Vec2d;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.AnalogEncoder;
import frc.robot.Constants;

/**
 * Container for one swerve module. Wraps two falcon500s: one for driving and one for steering.
 *
 * @author 2141 Spartonics
 */
public class QuailSwerveModule extends SwerveModuleBase {
  /** The PID id used to determine what PID settings to use. */
  /** The motor controlling the module's movement. */
  public final CANSparkMax drivingMotor;

  /** The motor controlling the module's rotation. */
  public final CANSparkMax steeringMotor;

  /** The can coder measuring the module's absolute rotaiton. */
  private final AnalogEncoder analogEncoder;

  /** The can coder's rotational offset. This value must be manually set through phoenix tuner. */
  public final double analogEncoderID;

  public double analogEncoderOffset;
  private SparkPIDController pidController;

  private double targetAngleAdjusted = 0;

  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;

  /**
   * @param driveMotor driving motor ID
   * @param steeringMotor steering motor ID
   * @param canCoder can coder ID
   * @param rotationDirection the steering motor's rotational direction, usually perpendicular to
   *     the center of the robot
   * @param canCoderOffset the can coder's rotational offset
   */
  public QuailSwerveModule(
      Vec2d position,
      int driveMotorID,
      int steeringMotorID,
      int analogEncoderID,
      double analogEncoderOffset) {
    // Call the super constructor.
    super(position, Constants.steeringRatio, Constants.driveRatio, true);

    this.analogEncoderOffset = analogEncoderOffset;
    this.analogEncoderID = analogEncoderID;

    // Initialize the motors and encoder.
    this.drivingMotor = new CANSparkMax(driveMotorID, MotorType.kBrushless);
    this.steeringMotor = new CANSparkMax(steeringMotorID, MotorType.kBrushless);
    this.analogEncoder = new AnalogEncoder(analogEncoderID);
    pidController = this.steeringMotor.getPIDController();

    // Setup PID
    kP = 0.2;
    kI = 0.00;
    kD = 0.25;
    kIz = 0;
    kFF = 0.000;
    kMaxOutput = 1;
    kMinOutput = -1;

    pidController.setP(kP);
    // pidController.setI(kI);
    pidController.setD(kD);
    // pidController.setIZone(kIz);
    // pidController.setFF(kFF);
    pidController.setOutputRange(kMinOutput, kMaxOutput);

    this.steeringMotor.setInverted(false);
    this.steeringMotor.setIdleMode(IdleMode.kBrake);
    this.steeringMotor.setSmartCurrentLimit(60);
    this.steeringMotor.burnFlash();

    this.drivingMotor.setIdleMode(IdleMode.kBrake);
    this.drivingMotor.setInverted(true);
    this.drivingMotor.setSmartCurrentLimit(60);
    this.drivingMotor.burnFlash();

    System.out.println("Finished initializing" + this.toString());
    this.reset();
    System.out.println("Reset module...");
  }

  /** Sets the module's angle to the desired angle */
  public void reset() {
    // Reset the motor rotations.
    this.steeringMotor
        .getEncoder()
        .setPosition(getAbsoluteEncoderAngle() * Constants.steeringRatio);
    this.currentAngle = (getAbsoluteEncoderAngle() * Constants.TWO_PI);
    this.drivingMotor.stopMotor();
    this.steeringMotor.stopMotor();
  }

  // returns rotations, 0 is x axis
  public double getAbsoluteEncoderAngle() {
    double currentPos = this.analogEncoder.getAbsolutePosition() - this.analogEncoderOffset;
    currentPos = (currentPos + 1) % 1;
    return currentPos;
  }

  /**
   * Returns the steering motor's angle in radians. Fetches the velocity in RPM from the motor and
   * converts it to radians per second.
   *
   * @return The steering motor's rotation in radians per second
   */
  public Vec2d getCurrentMovement() {
    double speed = this.drivingMotor.getEncoder().getVelocity();
    speed = speed / 60; // convert to seconds
    speed = speed / Constants.driveRatio; // convert to real wheel rotations
    speed = speed * Constants.wheelDiameter * Math.PI;
    return new Vec2d(this.getAbsoluteEncoderAngle() * Constants.TWO_PI, speed, false);
  }

  // @marcus: can you write a docstring here? I'm not sure what this does
  @Override
  public void setRawAngle(double angle) {
    this.targetAngleAdjusted = (angle / (2 * Math.PI)) * Constants.GEAR_RATIO_SWERVE;
    this.pidController.setReference(this.targetAngleAdjusted, CANSparkMax.ControlType.kPosition);
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
  public CANSparkMax getDriveMotor() {
    return this.drivingMotor;
  }

  /**
   * Getter for the steering motor.
   *
   * @return The steering motor.
   */
  public CANSparkMax getSteeringMotor() {
    return this.steeringMotor;
  }

  @Override
  public String toString() {
    // The class will be represented as "SwerveModule[Steering Motor ID = ?, Driving
    // Motor ID = ?, Cancoder ID = ?]"
    return "SwerveModule[Steering Motor ID = "
        + this.steeringMotor.getDeviceId()
        + ", Driving Motor ID = "
        + this.drivingMotor.getDeviceId()
        + ", Cancoder ID = "
        + this.analogEncoderID
        + "]";
  }

  /**
   * Create the telemetry for the swerve module.
   *
   * @param builder the telemetry builder to add the module's data to
   */
  public void initSendable(SendableBuilder builder) {
    String name = "SwerveModule[" + this.steeringMotor.getDeviceId() + "]";

    builder.addDoubleProperty(name + " Absolute Encoder", () -> getAbsoluteEncoderAngle(), null);
    builder.addDoubleProperty(
        name + " Motor Encoder",
        () -> steeringMotor.getEncoder().getPosition() / Constants.GEAR_RATIO_SWERVE,
        null);
  }
}
