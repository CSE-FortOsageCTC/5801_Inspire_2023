package frc.robot.autos;

import frc.robot.commands.AutoBalance;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class AutoBalanceAuto extends SequentialCommandGroup {
boolean fieldRelative = true;
boolean openLoop = true;


    public AutoBalanceAuto(Swerve s_Swerve){
        AutoBalance autoBalanceCommand = new AutoBalance(s_Swerve, fieldRelative, openLoop);
        addRequirements(s_Swerve);
        addCommands(
        autoBalanceCommand
        );
    }
}