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

public class A3B4C extends SequentialCommandGroup {
  
  public A3B4C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

  
    PathPlannerTrajectory A3B4 = PathPlanner.loadPath("A3B4", 3.0, 1.0);
    PathPlannerTrajectory B4C = PathPlanner.loadPath("B4C", 3.0, 1.0);
    
    addCommands(
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(A3B4, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Floor)),//move to and prepare arm for field cone
      new IntakeAuto(s_IntakeSubsystem, -0.5).withTimeout(0.1),
      new DriveForward(drive).withTimeout(1),//pick of field cone
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(B4C, false).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Default)),//move back to spawn and prepare arm to place cone
      new AutoBalanceSetup(drive, true, true).withTimeout(2),
      new AutoBalance(drive, true, true)
    );
  }
}
