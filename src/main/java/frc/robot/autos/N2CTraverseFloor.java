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

public class N2CTraverseFloor extends SequentialCommandGroup {
  
  public N2CTraverseFloor(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A2C = PathPlanner.loadPath("A2C", 3, 1);
    ArmPosition high = ArmPosition.High;
    ArmPosition highSequence = ArmPosition.HighSequence;
    ArmPosition highSequence1 = ArmPosition.HighSequence1;
    ArmPosition travel = ArmPosition.Travel;
    ArmPosition travelSequence = ArmPosition.TravelSequence;
    ArmPosition autoMidSequence = ArmPosition.AutoMidSequence;
    ArmPosition aMidSeq1 = ArmPosition.AutoMidSequence1;
    ArmPosition mid = ArmPosition.Mid;
    
    addCommands(
      new InstantCommand(() -> drive.gyro180()),
      new TraverseChargeStation(drive, true, true, false, false, false, 0).alongWith(new PositionArm(s_ArmSubsystem, List.of(travelSequence, travel))).withTimeout(5.35),
      new WaitCommand(0.25),
      new AutoBalanceSetup(drive, true, true).withTimeout(2.8),
      new AutoBalance(drive, true, true)
    );
  }
}