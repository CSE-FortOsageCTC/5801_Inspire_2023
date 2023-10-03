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

public class N1B4N1B3 extends SequentialCommandGroup {
  
  public N1B4N1B3(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {
    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A1B4 = PathPlanner.loadPath("1B4", 3, 1);
    PathPlannerTrajectory B4A1 = PathPlanner.loadPath("B41", 3, 1);
    PathPlannerTrajectory A1B3 = PathPlanner.loadPath("1B3", 3, 1);
    ArmPosition floorSequence = ArmPosition.FloorSequence;
    ArmPosition floor = ArmPosition.Floor;
    ArmPosition rotateWrist = ArmPosition.AutoWristRotate;
    ArmPosition midSequence = ArmPosition.MidSequence;
    addCommands(
        // USING withTimeout FOR THE INTAKE  
        new InstantCommand(() -> drive.gyro180()),

        drive.followTrajectoryCommand(A1B4, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(floorSequence, rotateWrist, floor))).withTimeout(4),//move to and prepare arm for field cone
        
        new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInTeleopConstant).withTimeout(0.000001),
        new DriveForward(drive).withTimeout(.75),//pick up field cone
        new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
        new DriveBackward(drive).withTimeout(.75),//return to PP position

        drive.followTrajectoryCommand(B4A1, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.High)).withTimeout(4),

        new AutoAutoAlign(drive, true, 0.35, 0.25).withTimeout(2),
        new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1),
        new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
        new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),

        drive.followTrajectoryCommand(A1B3, true).alongWith(new PositionArm(s_ArmSubsystem, List.of(midSequence, floorSequence, floor))).withTimeout(1)

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