package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {

  private TalonFX t1 = new TalonFX(21);
  private TalonFX t2 = new TalonFX(22);

  public Shooter() {
    TalonFXConfiguration bothConfiguration = new TalonFXConfiguration();
    t1.getConfigurator().apply(bothConfiguration);
    t2.getConfigurator().apply(bothConfiguration);

    t1.setNeutralMode(NeutralModeValue.Coast);
    t2.setNeutralMode(NeutralModeValue.Coast);

    t2.setInverted(true);
    t1.setInverted(false);
  }

  public void init() {
    TalonFXConfiguration bothConfiguration = new TalonFXConfiguration();
    t1.getConfigurator().apply(bothConfiguration);
    t2.getConfigurator().apply(bothConfiguration);

    t1.setNeutralMode(NeutralModeValue.Coast);
    t2.setNeutralMode(NeutralModeValue.Coast);

    t2.setInverted(true);
    t1.setInverted(false);
  }

  public void fullsend() {

    t1.setControl(new VoltageOut(7));
    t2.setControl(new VoltageOut(7));
  }

  public void stopMotors() {
    t1.stopMotor();
    t2.stopMotor();
  }

  public Command fullsendCommand() {
    return this.runOnce(
        () -> {
          this.fullsend();
        });
  }

  public Command stopMotorsCommand() {
    return this.runOnce(
        () -> {
          this.stopMotors();
        });
  }
}
