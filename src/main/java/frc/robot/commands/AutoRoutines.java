package frc.robot.commands;

import java.util.ArrayList;

import com.mineinjava.quail.util.geometry.Pose2d;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.QuailDriveTrain;
public final class AutoRoutines {

  private QuailDriveTrain driveTrain;
  private GenericEntry targetPoseX;
  private GenericEntry targetPoseY;
  private GenericEntry targetPoseHeading;


  public AutoRoutines(QuailDriveTrain driveTrain) {
    this.driveTrain = driveTrain;

    targetPoseX = Shuffleboard.getTab("Auto").add("Target Pose X", 0).getEntry();
    targetPoseY = Shuffleboard.getTab("Auto").add("Target Pose Y", 0).getEntry();
    targetPoseHeading = Shuffleboard.getTab("Auto").add("Heading", 0).getEntry();
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

  public Command driveToPose() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();

    double x = targetPoseX.getDouble(0);
    double y = targetPoseY.getDouble(0);
    double heading = targetPoseHeading.getDouble(0);

    points.add(new Pose2d(x,y,heading));

    return Commands.parallel(
      new RunPath(this.driveTrain, points)
    );
  }

}
