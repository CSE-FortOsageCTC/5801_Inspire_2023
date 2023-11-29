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

public class N2C extends SequentialCommandGroup {
  
  public N2C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A2C = PathPlanner.loadPath("A2C", 3, 1);
    ArmPosition high = ArmPosition.High;
    ArmPosition highSequence = ArmPosition.HighSequence;
    ArmPosition highSequence1 = ArmPosition.HighSequence1;
    ArmPosition travel = ArmPosition.Travel;
    ArmPosition travelSequence = ArmPosition.TravelSequence;
    ArmPosition midSequence = ArmPosition.MidSequence;
    
    addCommands(
      new InstantCommand(() -> drive.zeroGyro()),
      //new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(3),
      //new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1.5),
      ////new WaitCommand(0.2),
      //new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      //new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      //new PositionArm(s_ArmSubsystem, ArmPosition.Default).withTimeout(3),
      new PositionArm(s_ArmSubsystem, List.of(highSequence1, highSequence, high)).withDartSpeed(1).withTimeout(3.75), //6 timeout

      new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      //new TraverseChargeStation(drive, true, true, false, false, false, 0).alongWith(new PositionArm(s_ArmSubsystem, List.of(midSequence, travelSequence, travel))).withTimeout(6),
      //new WaitCommand(0.25),
      new AutoBalanceSetup(drive, true, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(midSequence, travelSequence, travel)).withDartSpeed(1)).withTimeout(2.5),
      new AutoBalance(drive, true, true)
    );
  }
}