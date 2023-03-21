package frc.robot.autos;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.commands.*;
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
      new PositionArm(s_ArmSubsystem, ArmPosition.High).withTimeout(5),
      //new WaitCommand(0.2),
      new IntakeAuto(s_IntakeSubsystem, 0.3).withTimeout(.5),//place cone on top bar
      new IntakeAuto(s_IntakeSubsystem, 0).withTimeout(0.1),
      drive.followTrajectoryCommand(A2C, true).alongWith(new PositionArm(s_ArmSubsystem, ArmPosition.Default)),//mobility and charge station
      //new DriveForward(drive).withTimeout(4),
      new TraverseChargeStation(drive, true, true, false, false, false, 0).withTimeout(8),
      new DriveForward(drive).withTimeout(0.25),
      new DriveBackward(drive).withTimeout(0.25),
      new AutoBalanceSetup(drive, true, true).withTimeout(2),
      new AutoBalance(drive, true, true)
    );
  }
}