package frc.robot.commands;

import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.util.geometry.Pose2d;
import frc.robot.subsystems.QuailDriveTrain;
import java.util.ArrayList;

public class AlignShooter extends RunPath {
  public Path path;

  public AlignShooter(QuailDriveTrain drivetrain) {
    super(drivetrain, new ArrayList<Pose2d>());
  }

  @Override
  public void initialize() {
    ArrayList<Pose2d> shooterPose = new ArrayList<Pose2d>();
    shooterPose.add(this.drivetrain.shooterPosition());
    this.path = new Path(shooterPose);
    System.out.println("======== initialize AlignShooter" + this.path.points.toString());

    super.initialize();
  }
}
