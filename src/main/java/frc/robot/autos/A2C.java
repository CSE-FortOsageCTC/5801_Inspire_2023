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
      // USING withTimeout FOR THE INTAKE  
      new InstantCommand(() -> drive.gyro180()),
      new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(2.75),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, Constants.AutoConstants.intakeInAutoConstant).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(A2C, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Default)).withTimeout(2.5),//mobility and charge station
      new AutoBalanceSetup(drive, true, true).withTimeout(2.5),
      new AutoBalance(drive, true, true)
    );
  }
}