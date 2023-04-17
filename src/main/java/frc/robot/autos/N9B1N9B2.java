package frc.robot.autos;

import java.util.List;

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

public class N9B1N9B2 extends SequentialCommandGroup {
  
  public N9B1N9B2(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A9B1 = PathPlanner.loadPath("9B1", 3, 1);
    PathPlannerTrajectory B1A9 = PathPlanner.loadPath("B19", 3, 1);
    PathPlannerTrajectory A9B2 = PathPlanner.loadPath("9B2", 3, 1);
    PathPlannerTrajectory B2A8 = PathPlanner.loadPath("B28", 3, 1);
    ArmPosition floorSequence = ArmPosition.FloorSequence;
    ArmPosition floor = ArmPosition.Floor;
    ArmPosition rotateWrist = ArmPosition.AutoWristRotate;
    ArmPosition midSequence = ArmPosition.MidSequence;
    addCommands(
        // USING withTimeout FOR THE INTAKE  
        new InstantCommand(() -> drive.gyro180()),

        drive.followTrajectoryCommand(A9B1, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(floorSequence, rotateWrist, floor))).withTimeout(4),//move to and prepare arm for field cone
        
        new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInTeleopConstant).withTimeout(0.000001),
        new DriveForward(drive).withTimeout(.75),//pick up field cone
        new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
        new DriveBackward(drive).withTimeout(.75),//return to PP position

        drive.followTrajectoryCommand(B1A9, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.High)).withTimeout(4),

        new AutoAutoAlign(drive, true, 0.35, 0.25).withTimeout(2),
        new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1),
        new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
        new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),

        drive.followTrajectoryCommand(A9B2, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(midSequence, floorSequence, floor))).withTimeout(1),
        new InstantCommand(() -> drive.invertInvertGyro())

        // new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInTeleopConstant).withTimeout(0.000001),
        // new DriveForward(drive).withTimeout(.5),//pick up field cone
        // new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
        // new DriveBackward(drive).withTimeout(.5),//return to PP position

        // drive.followTrajectoryCommand(B2A8, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Mid)),
        
        // new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
        // new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001)
    );
  }
}