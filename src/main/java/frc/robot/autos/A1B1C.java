package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.Swerve;

public class A1B1C extends SequentialCommandGroup {
  
  public A1B1C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A1B1 = PathPlanner.loadPath("A1B1", 3, 1);
    PathPlannerTrajectory B1C = PathPlanner.loadPath("B1C", 3, 1);

    
    addCommands(
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(5),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(A1B1, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new IntakeAuto(s_IntakeSubsystem, -0.5).withTimeout(0.1),
      new DriveForward(drive).withTimeout(1),//pick of field cone
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(B1C, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Default)),//move back to spawn and prepare arm to place cone
      new AutoBalanceSetup(drive, true, true).withTimeout(2),
      new AutoBalance(drive, true, true)
    );
  }
}
