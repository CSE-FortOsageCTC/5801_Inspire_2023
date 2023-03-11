package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class A3B4A3 extends SequentialCommandGroup {
  
  public A3B4A3(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

  
    PathPlannerTrajectory A3B4 = PathPlanner.loadPath("A3B4", 1.0, 1.0);
    PathPlannerTrajectory B4A3 = PathPlanner.loadPath("B4A3", 1.0, 1.0);
    
    addCommands(
      new PositionArm(s_ArmSubsystem, ArmPosition.High),
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3)),//place cone on top bar
      drive.followTrajectoryCommand(A3B4, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, 0.3)),//pick of field cone
      drive.followTrajectoryCommand(B4A3, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)),//move back to spawn and prepare arm to place cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3))//place cone on mid bar
    );
  }
}
