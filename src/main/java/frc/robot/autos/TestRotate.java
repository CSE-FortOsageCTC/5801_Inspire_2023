package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class TestRotate extends SequentialCommandGroup {
  
  public TestRotate(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory TestRotate = PathPlanner.loadPath("TestRotate", 1.0, 1.0);

    
    addCommands(
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.gyro180()),
      drive.followTrajectoryCommand(TestRotate, true)

      
      /* USING WaitCommand FOR THE INTAKE
      new PositionArm(s_ArmSubsystem, ArmPosition.High),
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3)),//place cone on top bar
      drive.followTrajectoryCommand(A1B1, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, 0.3)),//pick of field cone
      drive.followTrajectoryCommand(B1A1, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)),//move back to spawn and prepare arm to place cone
      new WaitCommand(.5).raceWith(new IntakeAuto(s_IntakeSubsystem, -0.3))//place cone on mid bar
      */
    );
  }
}