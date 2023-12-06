package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class TestAuto extends SequentialCommandGroup {



    public TestAuto(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {
        addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
        PathPlannerTrajectory testingPath = PathPlanner.loadPath("TestingPath", 3, 1);
        addCommands(
            new InstantCommand(() -> drive.gyro180()),    
            drive.followTrajectoryCommand(testingPath, true));
    }

}
