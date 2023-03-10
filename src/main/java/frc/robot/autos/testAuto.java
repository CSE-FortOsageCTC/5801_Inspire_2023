package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Swerve;

public class testAuto extends SequentialCommandGroup {

  public testAuto(Swerve drive) {

    PathPlannerTrajectory TestPathmar3 = PathPlanner.loadPath("TestPath", 1.0, 1.0);
    addCommands(
      drive.followTrajectoryCommand(TestPathmar3, true)
    );
  }
}
