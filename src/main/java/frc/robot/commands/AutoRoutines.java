package frc.robot.commands;

import java.util.ArrayList;

import com.mineinjava.quail.util.geometry.Pose2d;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.QuailDriveTrain;

public final class AutoRoutines {

  private QuailDriveTrain driveTrain;

  public AutoRoutines(QuailDriveTrain driveTrain) {
    this.driveTrain = driveTrain;
  }

  public Command defaultAuto() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0,0,0));
    points.add(new Pose2d(0, 20,0));
    points.add(new Pose2d(0, 0,0));


    return Commands.parallel(
      new RunPath(this.driveTrain, points)
    );
  }

  public Command driveForward10Feet() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0,0,0));
    points.add(new Pose2d(0, 10,0));

    return Commands.parallel(
      new RunPath(this.driveTrain, points)
    );
  }

}
