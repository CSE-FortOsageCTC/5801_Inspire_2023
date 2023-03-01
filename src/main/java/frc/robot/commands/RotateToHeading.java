package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

public class RotateToHeading extends CommandBase{
    
    private final Swerve s_Swerve;
    private final AutoRotateUtil rotateUtil;

    private int m_angle;
    public RotateToHeading(Swerve s_Swerve, int angle) {
        this.s_Swerve = s_Swerve;
        this.m_angle = angle == 0?360:angle;
        this.rotateUtil = new AutoRotateUtil(s_Swerve, this.m_angle );

   }


   @Override
   public void execute () {

    double speed = rotateUtil.calculateRotationSpeed();

    s_Swerve.drive(new Translation2d(0, 0), speed * (Constants.Swerve.maxAngularVelocity), true, true);
    SmartDashboard.putNumber("Speed", speed);

    

   }


   @Override
   public void end (boolean interupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
   }
}
