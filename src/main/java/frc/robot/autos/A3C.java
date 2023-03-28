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

public class A3C extends SequentialCommandGroup {
  
  public A3C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A3C = PathPlanner.loadPath("A3C", 3, 1);
    
    addCommands(
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(4.5),
      new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1.5),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.25),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      new PositionArm(s_ArmSubsystem, ArmPosition.Default).withTimeout(0.1),
      drive.followTrajectoryCommand(A3C, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Default)),//mobility and charge station
      new AutoBalanceSetup(drive, true, true).withTimeout(2),
      new AutoBalance(drive, true, true)
    );
  }
}