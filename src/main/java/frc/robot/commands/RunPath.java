package frc.robot.commands;

import java.util.ArrayList;

import com.mineinjava.quail.RobotMovement;
import com.mineinjava.quail.pathing.ConstraintsPair;
import com.mineinjava.quail.pathing.Path;
import com.mineinjava.quail.pathing.PathFollower;
import com.mineinjava.quail.util.MiniPID;
import com.mineinjava.quail.util.geometry.Pose2d;
import com.mineinjava.quail.util.geometry.Vec2d;

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
        ConstraintsPair translationPair = new ConstraintsPair(30, 60);
        ConstraintsPair rotationPair = new ConstraintsPair(0.5, 1);

        // TODO: Move to constants + tune
        this.pidController = new MiniPID(6.5, 0.0, 0.2);
        this.pidController.setF(0);
        

        this.pathfollower = new PathFollower(this.drivetrain.getOdometry(), this.path, translationPair, rotationPair,
                this.pidController, 1, 5, 1, 15);

        addRequirements(drivetrain);

        System.out.println("Constructed runPath Command w/ PF: " + pathfollower);
    }

    @Override
    public void initialize() {
        drivetrain.stop();
        this.path.currentPointIndex = 0;
        System.out.println("Initialized RunPath Command...");
    }

    @Override
    public void execute() {
        RobotMovement nextMovement = pathfollower.calculateNextDriveMovement();
        Vec2d newTranslation = (new Vec2d(nextMovement.translation.x / 200,
                nextMovement.translation.y / 200));
        double rotation = nextMovement.rotation;
        drivetrain.move(new RobotMovement(rotation / 20, newTranslation),
                this.drivetrain.odometry.theta);
    }

    @Override
    public boolean isFinished() {
        return this.pathfollower.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Run path completed - was interrupted: " + interrupted);
        drivetrain.stop();
        super.end(interrupted);
    }
}
