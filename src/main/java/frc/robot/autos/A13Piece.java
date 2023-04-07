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

public class A13Piece extends SequentialCommandGroup {
  
  public A13Piece(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A13Piece = PathPlanner.loadPath("A13Piece", 4, 3);
    PathPlannerTrajectory B13Piece = PathPlanner.loadPath("B13Piece", 4, 3);
    PathPlannerTrajectory A13rdPiece = PathPlanner.loadPath("A13rdPiece", 4, 3);
    PathPlannerTrajectory B13rdPiece = PathPlanner.loadPath("B13rdPiece", 4, 3);
    
    addCommands(
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.zeroGyro()), // start facing towards field
      drive.followTrajectoryCommand(A13Piece, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)).alongWith(new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant)),
      new IntakeAuto(s_IntakeSubsystem, 0),
      drive.followTrajectoryCommand(B13Piece, false),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(0.5),
      drive.followTrajectoryCommand(A13rdPiece, false).alongWith(new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant)),
      new IntakeAuto(s_IntakeSubsystem, 0),
      drive.followTrajectoryCommand(B13rdPiece, false),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(0.5)
    );
  }
}