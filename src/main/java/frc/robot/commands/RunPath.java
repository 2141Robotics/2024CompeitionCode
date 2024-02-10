package frc.robot.commands;

import java.util.ArrayList;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.localization.SwerveOdometry;
import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.components.QuailSwerveDrive;

public class RunPath extends Command {
    PathFollower pathfollower;
    Path path;
    QuailSwerveDrive drivetrain;
    ArrayList<Pose2d> points = new ArrayList<Pose2d>();
    SwerveOdometry odometry;

    public RunPath(PathFollower pathFollower, QuailSwerveDrive drivetrain, ArrayList<Pose2d> points, SwerveOdometry odometry){
        super();
        // this.points = points;
        // this.drivetrain = drivetrain;

        // this.pathfollower = pathFollower;
        // this.odometry = odometry;
    }
    
    // @Override
    // public void initialize() {
    //     super.initialize();
    //     drivetrain.softResetMotors();
    //     drivetrain.stop();
    //     System.out.println("auto commmand start");
    //     this.path = new Path(points);
    //     this.pathfollower.setPath(path);
        
    // }

    // @Override
    // public void execute(){
	// 	RobotMovement nextMovement = pathfollower.calculateNextDriveMovement();		
	// 	Vec2d newTranslation = (new Vec2d(nextMovement.translation.x/200, nextMovement.translation.y/200));
	// 	double rotation = nextMovement.rotation;

	// 	SmartDashboard.putNumber("PID output", rotation);

	// 	SmartDashboard.putNumber("autoX", nextMovement.translation.x);
	// 	SmartDashboard.putNumber("autoY", nextMovement.translation.y);

	// 	drivetrain.move(new RobotMovement(rotation/20, newTranslation), this.odometry.theta);
	// }

    // @Override
    // public boolean isFinished() {
    //     return this.pathfollower.isFinished();
    // }
    // @Override
    // public void end(boolean interrupted) {
    //     System.out.println("auto command end");
    //     drivetrain.stop();
    //     super.end(interrupted);
    // }

}
