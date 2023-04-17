package frc.robot.autos;

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

public class A2C extends SequentialCommandGroup {
  
  public A2C(Swerve drive, ArmSubsystem s_ArmSubsystem, IntakeSubsystem s_IntakeSubsystem) {

    addRequirements(drive, s_ArmSubsystem, s_IntakeSubsystem);
    PathPlannerTrajectory A2C = PathPlanner.loadPath("A2C", 3, 1);
    
    addCommands(
      new InstantCommand(() -> drive.gyro180()),
      //new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(3),
      //new PositionArm(s_ArmSubsystem, ArmPosition.HighPlace).withTimeout(1.5),
      //new WaitCommand(0.2),
      //new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeOutAutoConstant).withTimeout(.5),//place cone on top bar
      //new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.000001),
      //new PositionArm(s_ArmSubsystem, ArmPosition.Default).withTimeout(0.75),
      drive.followTrajectoryCommand(A2C, true),//mobility and charge station
      new AutoBalSetupInvert(drive, true, true).withTimeout(1),
      new AutoBalance(drive, true, true)
    );
  }
}