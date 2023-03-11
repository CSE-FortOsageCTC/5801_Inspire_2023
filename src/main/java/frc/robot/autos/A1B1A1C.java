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

public class A1B1A1C extends SequentialCommandGroup {
  
  public A1B1A1C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

  
    PathPlannerTrajectory A1B1 = PathPlanner.loadPath("A1B1", 1.0, 1.0);
    PathPlannerTrajectory B1A1 = PathPlanner.loadPath("B1A1", 1.0, 1.0);
    PathPlannerTrajectory A1C = PathPlanner.loadPath("A1C", 1.0, 1.0);
    AutoBalanceSetup autoBalanceSetupCommand = new AutoBalanceSetup(drive, true, true);
    AutoBalance autoBalanceCommand = new AutoBalance(drive, true, true);
    
    addCommands(
      new PositionArm(s_ArmSubsystem, ArmPosition.High),
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3)),//place cone on top bar
      drive.followTrajectoryCommand(A1B1, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, 0.3)),//pick of field cone
      drive.followTrajectoryCommand(B1A1, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)),//move back to spawn and prepare arm to place cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3)),//place cone on mid bar
      drive.followTrajectoryCommand(A1C, false).alongWith(new PositionArm(s_ArmSubsystem, null)),//move from community to charge station
      autoBalanceSetupCommand,//setup for autobalance
      autoBalanceCommand//balance
    );
  }
}
