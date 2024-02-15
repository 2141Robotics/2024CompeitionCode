package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ServoController extends SubsystemBase{
    public Servo servo = new Servo (0);
    public Command setEnableShooting() {
        return Commands.runOnce(() -> {
            this.servo.set(0);
        }, this);
    }public Command setDisableShooting() {
        return Commands.runOnce(() -> {
            this.servo.set(0.5);
        }, this);

    }
}