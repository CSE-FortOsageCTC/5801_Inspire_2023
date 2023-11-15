package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class A3B4A3 extends SequentialCommandGroup {
  
  public A3B4A3(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A3B4 = PathPlanner.loadPath("A3B4", 3, 1);
    PathPlannerTrajectory B4A3 = PathPlanner.loadPath("B4A3", 3, 1);
    PathPlannerTrajectory Left18 = PathPlanner.loadPath("Left18inches", 1.5, 1.0);
    
    addCommands(
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(3),
      new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1.5),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      new PositionArm(s_ArmSubsystem, ArmPosition.Default).withTimeout(0.75),
      drive.followTrajectoryCommand(A3B4, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new RotateToHeading(drive, 0).withTimeout(2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInTeleopConstant).withTimeout(0.000001),
      new DriveForward(drive).withTimeout(1),//pick of field cone
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001)//,
      /*drive.followTrajectoryCommand(B4A3, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)),//move back to spawn and prepare arm to place cone
      new AutoAlign(drive, false).withTimeout(2.5),
      drive.followTrajectoryCommand(Left18, false),
      new IntakeAuto(s_IntakeSubsystem, 0.3).withTimeout(.5),//place cone on mid bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1)*/
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