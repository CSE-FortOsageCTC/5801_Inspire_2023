package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.Constants.AutoConstants.ArmPosition.ArmMotor;
import frc.robot.subsystems.ArmSubsystem;

public class PositionArm extends CommandBase{
    
    private final ArmSubsystem s_Arm;
    private final ArmPosition position;
    private double wristSetpoint = 0;
    private double elbowSetpoint = 0;
    private double extensionSetpoint = 0;
    private double shoulderSetpoint = 0;

    //private String m_position;
    public PositionArm(ArmSubsystem s_Arm, ArmPosition position) {
        this.s_Arm = s_Arm;
        this.position = position;

        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        elbowSetpoint = position.getSetpoint(ArmMotor.Elbow);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        shoulderSetpoint = position.getSetpoint(ArmMotor.Shoulder);

        
        //SmartDashboard.putNumber("Wrist Setpoint", 0.48);
        SmartDashboard.putNumber("Wrist P Value", 5.5);
        SmartDashboard.putNumber("Wrist I Value", 0);
        SmartDashboard.putNumber("Wrist D Value", 0);

        //SmartDashboard.putNumber("Elbow Setpoint", 0.75);
        SmartDashboard.putNumber("Elbow P Value", 10);
        SmartDashboard.putNumber("Elbow I Value", 0);
        SmartDashboard.putNumber("Elbow D Value", 0);

        //SmartDashboard.putNumber("Extension Setpoint", 0);
        SmartDashboard.putNumber("Extension P Value", 0);
        SmartDashboard.putNumber("Extension I Value", 0);
        SmartDashboard.putNumber("Extension D Value", 0);

        //SmartDashboard.putNumber("Shoulder Setpoint", 0);
        SmartDashboard.putNumber("Shoulder P Value", 0);
        SmartDashboard.putNumber("Shoulder I Value", 0);
        SmartDashboard.putNumber("Shoulder D Value", 0);
        
    }

   @Override
   public void initialize() {
   }

   @Override
   public void execute () {
    
    //double wristSetpoint = SmartDashboard.getNumber("Wrist Setpoint", 0.48);
    //double elbowSetpoint = SmartDashboard.getNumber("Elbow Setpoint", 0.75);
    //double extensionSetpoint = SmartDashboard.getNumber("Extension Setpoint", 0);
    //double shoulderSetpoint = SmartDashboard.getNumber("Shoulder Setpoint", 0);



    //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
    //s_Arm.moveWrist(speed);
    s_Arm.wristPID(wristSetpoint);
    s_Arm.elbowPID(elbowSetpoint);
    s_Arm.extensionPID(extensionSetpoint);
    s_Arm.shoulderPID(shoulderSetpoint);
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