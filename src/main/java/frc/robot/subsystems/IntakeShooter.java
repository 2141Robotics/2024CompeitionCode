package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeShooter extends SubsystemBase {

  private TalonFX s1 = new TalonFX(21);
  private TalonFX s2 = new TalonFX(22);

  private CANSparkMax i1 = new CANSparkMax(41, MotorType.kBrushless);
  private CANSparkMax i2 = new CANSparkMax(42, MotorType.kBrushless);

  // Limit switch for the intake
  private DigitalInput intakeLimitSwitch = new DigitalInput(0);

  public IntakeShooter() {
    // config shooter motors
    TalonFXConfiguration bothConfiguration = new TalonFXConfiguration();
    s1.getConfigurator().apply(bothConfiguration);
    s2.getConfigurator().apply(bothConfiguration);

    s1.setNeutralMode(NeutralModeValue.Coast);
    s2.setNeutralMode(NeutralModeValue.Coast);

    s2.setInverted(true);
    s1.setInverted(false);

    // config intake motors
    i1.restoreFactoryDefaults();
    i2.restoreFactoryDefaults();

    i1.setIdleMode(IdleMode.kBrake);
    i2.setIdleMode(IdleMode.kBrake);

    i1.setInverted(true);
    i2.setInverted(false);

    i1.setSmartCurrentLimit(12);
    i2.setSmartCurrentLimit(12);

    i1.burnFlash();
    i2.burnFlash();
  }

  public boolean hasNote() {
    return intakeLimitSwitch.get();
  }

  public void retractIntakeMotors() {
    i1.set(Constants.INTAKE_RETRACT_SPEED);
    i2.set(Constants.INTAKE_RETRACT_SPEED);
  }

  public void startIntakeMotors() {
    i1.set(Constants.INTAKE_SPEED);
    i2.set(Constants.INTAKE_SPEED);
  }

  public void stopIntakeMotors() {
    i1.stopMotor();
    i2.stopMotor();
  }

  public void startShooterMotors() {
    s1.setControl(new VoltageOut(Constants.SHOOTER_VOLTAGE));
    s2.setControl(new VoltageOut(Constants.SHOOTER_VOLTAGE));
  }

  public void stopShooterMotors() {
    s1.stopMotor();
    s2.stopMotor();
  }

  public Command startShooterMotorsCommand() {
    return this.runOnce(
        () -> {
          this.startShooterMotors();
        });
  }

  public Command stopShooterMotorsCommand() {
    return this.runOnce(
        () -> {
          this.stopShooterMotors();
        });
  }

  public Command startIntakeMotorsCommand() {
    return this.runOnce(
        () -> {
          this.startIntakeMotors();
        });
  }

  public Command stopIntakeMotorsCommand() {
    return this.runOnce(
        () -> {
          this.stopIntakeMotors();
        });
  }

  public Command retractIntakeMotorsCommand() {
    return this.runOnce(
        () -> {
          this.retractIntakeMotors();
        });
  }

  public Command shoot() {
    /*
    i1.set(0.35);
    i2.set(0.35);
    s1.set(1);
    s2.set(1);
    */
    System.out.println("SHOOTING");
    return this.retractIntakeMotorsCommand() // start retracting note
        .andThen(Commands.waitSeconds(Constants.SHOOTER_REVERSE_TIME)) // wait
        .andThen(this.stopIntakeMotorsCommand()) // stop retracting
        .andThen(this.startShooterMotorsCommand()) // spin up shooter motors
        .andThen(Commands.waitSeconds(Constants.SHOOTER_WAIT_TIME)) // wait
        .andThen(this.startIntakeMotorsCommand()) // feed note into shooter
        .andThen(Commands.waitSeconds(Constants.SHOOTER_SHOOT_TIME)) // wait
        .finallyDo(
            (boolean _interrupted) -> { // stop motors on end
              this.stopIntakeMotors();
              this.stopShooterMotors();
            });
  }
}
