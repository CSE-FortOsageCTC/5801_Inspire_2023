package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class N1Mobility extends SequentialCommandGroup {
  
  public N1Mobility(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory N1B4rotate = PathPlanner.loadPath("1B4rotate", 3, 1);
    ArmPosition high = ArmPosition.High;
    ArmPosition highSequence = ArmPosition.HighSequence;
    ArmPosition highSequence1 = ArmPosition.HighSequence1;
    ArmPosition travel = ArmPosition.Travel;
    ArmPosition travelSequence = ArmPosition.TravelSequence;
    ArmPosition midSequence = ArmPosition.MidSequence;
    ArmPosition floorSequence = ArmPosition.FloorSequence;
    ArmPosition floor = ArmPosition.Floor;
    ArmPosition rotateWrist = ArmPosition.AutoWristRotate;
    
    addCommands(
      new InstantCommand(() -> drive.zeroGyro()),
      //new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(3),
      //new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1.5),
      ////new WaitCommand(0.2),
      //new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      //new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      //new PositionArm(s_ArmSubsystem, ArmPosition.Default).withTimeout(3),
      new PositionArm(s_ArmSubsystem, List.of(highSequence1, highSequence, high)).withDartSpeed(1).withTimeout(4), //6 timeout
      new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      drive.followTrajectoryCommand(N1B4rotate, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(midSequence, floorSequence, floor))).withTimeout(4),//move to and prepare arm for field cone
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInTeleopConstant).withTimeout(0.000001),
      new DriveForward(drive).withTimeout(.75),//pick up field cone
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      new DriveBackward(drive).withTimeout(.75)//return to PP position
    );
  }
}