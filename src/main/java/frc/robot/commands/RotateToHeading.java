package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;

public class RotateToHeading extends CommandBase{
    
    private final Swerve s_Swerve;
    private final PIDController pidController;

    private int m_angle;
    public RotateToHeading(Swerve s_Swerve, int angle) {
        this.s_Swerve = s_Swerve;
        this.m_angle = angle == 0?360:angle;

        //SmartDashboard.putNumber("Angle", this.m_angle);
        this.pidController = new PIDController(0, 0, 0);

        pidController.setTolerance(0.675);
        pidController.setSetpoint(this.m_angle);
        SmartDashboard.putNumber("kP", 0.007);
        SmartDashboard.putNumber("kI", 0.0025);
        SmartDashboard.putNumber("kD", 0.001);
   }

   @Override
   public void initialize() {
    pidController.reset();
   }


   @Override
   public void execute () {
    
    double kP = SmartDashboard.getNumber("kP", 0.0);
    double kI = SmartDashboard.getNumber("kI", 0.0);
    double kD = SmartDashboard.getNumber("kD", 0.0);

    this.pidController.setP(kP);
    this.pidController.setI(kI);
    this.pidController.setD(kD);

    double headingError = this.m_angle - s_Swerve.getYawDouble();
    if (headingError > 180) {
        headingError -= 360;
    }
    if (headingError < -180) {
        headingError += 360;
    }
    double speed = pidController.calculate(s_Swerve.getYawDouble(), s_Swerve.getYawDouble() + headingError);
    s_Swerve.drive(new Translation2d(0, 0), speed * (Constants.Swerve.maxAngularVelocity), true, true);
    SmartDashboard.putNumber("Speed", speed);

    

   }

   @Override
   public boolean isFinished () {
    return pidController.atSetpoint();
    //double speed = pidController.calculate(s_Swerve.getYawDouble());
    //return speed < 0.1;
   }

   @Override
   public void end (boolean interupted) {
    s_Swerve.drive(new Translation2d(0, 0), 0, true, true);
   }
}
