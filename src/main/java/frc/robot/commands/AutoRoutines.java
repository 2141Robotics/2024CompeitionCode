package frc.robot.commands;

import com.mineinjava.quail.util.geometry.Pose2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeShooter;
import frc.robot.subsystems.QuailDriveTrain;
import java.util.ArrayList;

public final class AutoRoutines {

  private QuailDriveTrain driveTrain;
  private IntakeShooter intakeShooter;

  private GenericEntry targetPoseX;
  private GenericEntry targetPoseY;
  private GenericEntry targetPoseHeading;
  ArrayList<Pose2d> shootingPos = new ArrayList<Pose2d>();

  public AutoRoutines(QuailDriveTrain driveTrain, IntakeShooter shooter) {
    // ''this.shootingPos,add(new Pose2d(-50,-230,0)); // blue alliance // DON"T USE
    // ''this.shootingPos,add(new Pose2d(50, -230, 0)); // red alliance // DONT USE

    this.shootingPos.add(new Pose2d(-45, -225, 0)); // blue alliance //
    // this.shootingPos.add(new Pose2d(45, -225, 0)); // red alliance
    this.driveTrain = driveTrain;
    this.intakeShooter = shooter;

    targetPoseX = Shuffleboard.getTab("Autonomous").add("TargetPoseX", 0).getEntry();
    targetPoseY = Shuffleboard.getTab("Autonomous").add("TargetPoseY", 0).getEntry();
    targetPoseHeading = Shuffleboard.getTab("Autonomous").add("Heading", 0).getEntry();
  }

  public Command noop() {
    return Commands.parallel();
  }

  public Command lineUpShooter() {
    return new RunPath(this.driveTrain, shootingPos);
  }

  public Command shootPreloadedNote() {
    return Commands.sequence(
        this.driveTrain.resetGyroCommand(),
        this.driveTrain.resetModulesCommand(),
        new WaitCommand(Constants.AUTO_WAIT_TIME),
        lineUpShooter(),
        this.driveTrain.stopCommand(),
        new WaitCommand(0.3),
        this.intakeShooter.shoot());
  }

  public Command noVisionTaxi() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0, 60, 0));

    return Commands.sequence(
        this.driveTrain.resetGyroCommand(),
        this.driveTrain.resetModulesCommand(),
        new RunPath(this.driveTrain, points));
  }

  public Command defaultAuto() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0, 0, 0));
    points.add(new Pose2d(0, 36, Math.PI / 4));
    points.add(new Pose2d(-36, 36, Math.PI / 2));
    points.add(new Pose2d(36, 72, Math.PI / 4));
    points.add(new Pose2d(0, 0, 0));

    return Commands.sequence(
        this.driveTrain.resetModulesCommand(), new RunPath(this.driveTrain, points));
  }

  public Command driveForward10Feet() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    points.add(new Pose2d(0, 0, 0));
    points.add(new Pose2d(0, 10, 0));

    return Commands.parallel(
        this.driveTrain.resetModulesCommand(), new RunPath(this.driveTrain, points));
  }

  /**
   * TODO: This right now has a bug since the initial pose is set when the command is created. We
   * need to set the pose when the command is run.
   *
   * @return
   */
  public Command driveToPose() {
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();

    double x = targetPoseX.getDouble(0);
    double y = targetPoseY.getDouble(0);
    System.out.println("Moving in auto to pose: " + x + " , " + y);
    double heading = targetPoseHeading.getDouble(0);
    // points.add(driveTrain.odometry.getPose());
    points.add(new Pose2d(x, y, heading));

    return Commands.sequence(
        this.driveTrain.resetGyroCommand(),
        // this.driveTrain.resetModulesCommand(),
        new RunPath(this.driveTrain, points));
  }
}
