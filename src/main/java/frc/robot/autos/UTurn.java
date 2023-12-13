package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class UTurn extends SequentialCommandGroup {

    public UTurn(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem){
        PathPlannerTrajectory backright = PathPlanner.loadPath("backright", 3, 1);
        PathPlannerTrajectory forward = PathPlanner.loadPath("forward", 3, 1);

        addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);

        addCommands(
            new InstantCommand(()-> drive.zeroGyro()),
            drive.followTrajectoryCommand(backright, true),
            drive.followTrajectoryCommand(forward, false)
            );
    }
}
