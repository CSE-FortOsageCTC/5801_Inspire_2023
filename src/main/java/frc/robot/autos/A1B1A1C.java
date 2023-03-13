package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class A1B1A1C extends SequentialCommandGroup {
  
  public A1B1A1C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A1B1 = PathPlanner.loadPath("A1B1", 3, 1);
    PathPlannerTrajectory B1A1 = PathPlanner.loadPath("B1A1", 3, 1);
    PathPlannerTrajectory Left18 = PathPlanner.loadPath("Left18inches", 1.5, 1.0);
    
    addCommands(
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, 0.3).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(A1B1, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new IntakeAuto(s_IntakeSubsystem, -0.5).withTimeout(0.1),
      new DriveForward(drive).withTimeout(1),//pick of field cone
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(B1A1, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)).withTimeout(2),//move back to spawn and prepare arm to place cone
      new AutoAlign(drive, false).withTimeout(2.5),
      drive.followTrajectoryCommand(Left18, false),
      new IntakeAuto(s_IntakeSubsystem, 0.3).withTimeout(.5),//place cone on mid bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1)
    );
  }
}
