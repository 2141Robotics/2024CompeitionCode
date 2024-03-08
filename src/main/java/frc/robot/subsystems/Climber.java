package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
  private TalonFX t1 = new TalonFX(31);
  private CANSparkMax t2 = new CANSparkMax(32, MotorType.kBrushless);

  public Climber() {
    TalonFXConfiguration talonConfig = new TalonFXConfiguration();
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
    talonConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = 0;
    t1.getConfigurator().apply(talonConfig);

    t2.restoreFactoryDefaults();
    t2.setSoftLimit(SoftLimitDirection.kReverse, 0);
    t1.setNeutralMode(NeutralModeValue.Brake);
    t2.setIdleMode(IdleMode.kBrake);

    t1.setInverted(false);
    t2.setInverted(false);
  }

  public void setMotorSpeeds(double s1, double s2) {
    t1.set(s1);
    t2.set(s2);
  }

  public void stopMotors() {
    t1.stopMotor();
    t2.stopMotor();
  }

  public void zeroMotors() {
    System.out.println("Resetting climber motors");
    t1.setPosition(0);
    t2.getEncoder().setPosition(0);
  }

  public Command zeroMotorsCommand() {
    return this.runOnce(
        () -> {
          this.zeroMotors();
        });
  }

  public Command stopMotorsCommand() {
    return this.runOnce(
        () -> {
          this.stopMotors();
        });
  }
}
