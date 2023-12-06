package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class RandiPath extends SequentialCommandGroup {
    public RandiPath(Swerve drive) {
        addRequirements(drive);
        PathPlannerTrajectory RLBPath = PathPlanner.loadPath("RLBPath", 3, 1);
        addCommands(
            drive.followTrajectoryCommand(RLBPath, true)
        );
    }
}
