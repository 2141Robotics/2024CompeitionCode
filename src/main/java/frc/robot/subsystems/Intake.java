package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
    
    Servo servo;

    public Intake() {
        servo = new Servo(2);
    }

    public void setServoPosition(double pos) {
        servo.set(pos);
    }

}
