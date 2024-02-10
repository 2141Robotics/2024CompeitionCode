package frc.robot.commands;

import java.util.ArrayList;

import com.mineinjava.quail.pathing.ConstraintsPair;
import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.QuailDriveTrain;

public class RunPath extends Command {
    private QuailDriveTrain drivetrain;
    private PathFollower pathfollower;
    private Path path;
    private MiniPID pidController;;

    public RunPath(QuailDriveTrain drivetrain, ArrayList<Pose2d> points) {
        this.path = new Path(points);
        this.drivetrain = drivetrain;

        // TODO: Put units on these
        ConstraintsPair translationPair = new ConstraintsPair(200, 300);
        ConstraintsPair rotationPair = new ConstraintsPair(0.1, 1);

        // TODO: Move to constants + tune
        this.pidController = new MiniPID(0.1, 0.0, 0.0);
        this.pidController.setF(0);
        ;

        this.pathfollower = new PathFollower(this.drivetrain.getOdometry(), this.path, translationPair, rotationPair,
                this.pidController, 3, 4, 1, 15);

        addRequirements(drivetrain);

        System.out.println("Constructed runPath Command w/ PF: " + pathfollower);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        System.out.println("Initialized RunPath Command...");
    }

    // @Override
    // public void execute(){
    // RobotMovement nextMovement = pathfollower.calculateNextDriveMovement();
    // Vec2d newTranslation = (new Vec2d(nextMovement.translation.x/200,
    // nextMovement.translation.y/200));
    // double rotation = nextMovement.rotation;

    // SmartDashboard.putNumber("PID output", rotation);

    // SmartDashboard.putNumber("autoX", nextMovement.translation.x);
    // SmartDashboard.putNumber("autoY", nextMovement.translation.y);

    // drivetrain.move(new RobotMovement(rotation/20, newTranslation),
    // this.odometry.theta);
    // }

    @Override
    public boolean isFinished() {
        return true;
        // return this.pathfollower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Run path completed - was interrupted: " + interrupted);
        drivetrain.stop();
        super.end(interrupted);
    }
}
