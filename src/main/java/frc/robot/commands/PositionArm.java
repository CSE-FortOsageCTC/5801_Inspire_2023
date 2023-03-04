package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Swerve;

import edu.wpi.first.math.controller.PIDController;

public class PositionArm extends CommandBase{
    
    private final ArmSubsystem s_Arm;

    //private String m_position;
    public PositionArm(ArmSubsystem s_Arm/*, String position*/) {
        this.s_Arm = s_Arm;
        //this.m_position = position;

        SmartDashboard.putNumber("Wrist Setpoint", 0.48);
        SmartDashboard.putNumber("Elbow Setpoint", 0.75);
        SmartDashboard.putNumber("Wrist P Value", 5.5);
        SmartDashboard.putNumber("Wrist I Value", 0);
        SmartDashboard.putNumber("Wrist D Value", 0);
        SmartDashboard.putNumber("Elbow P Value", 10);
        SmartDashboard.putNumber("Elbow I Value", 0);
        SmartDashboard.putNumber("Elbow D Value", 0);
    }

   @Override
   public void initialize() {
   }

   @Override
   public void execute () {
    
    double wristSetpoint = SmartDashboard.getNumber("Wrist Setpoint", 0.48);
    double elbowSetpoint = SmartDashboard.getNumber("Elbow Setpoint", 0.75);

    //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
    //s_Arm.moveWrist(speed);
    s_Arm.wristPIDCommand(wristSetpoint);
    s_Arm.elbowPIDCommand(elbowSetpoint);
   }

   @Override
   public boolean isFinished () {
    //return pidController.atSetpoint();
    return false;
   }

   @Override
   public void end (boolean interupted) {
   }
}