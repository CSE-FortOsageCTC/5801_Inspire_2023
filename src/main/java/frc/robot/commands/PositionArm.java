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
    private double extensionTimer = 0;

    //private String m_position;
    public PositionArm(ArmSubsystem s_Arm, ArmPosition position) {
        this.s_Arm = s_Arm;
        this.position = position;

        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        elbowSetpoint = position.getSetpoint(ArmMotor.Elbow);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        shoulderSetpoint = position.getSetpoint(ArmMotor.Shoulder);
        
        //SmartDashboard.putNumber("Wrist Setpoint", 0.48);
        SmartDashboard.putNumber("Wrist P Value", Constants.AutoConstants.wristP);
        SmartDashboard.putNumber("Wrist I Value", Constants.AutoConstants.wristI);
        SmartDashboard.putNumber("Wrist D Value", Constants.AutoConstants.wristD);

        //SmartDashboard.putNumber("Elbow Setpoint", 0.75);
        SmartDashboard.putNumber("Elbow P Value", Constants.AutoConstants.elbowP);
        SmartDashboard.putNumber("Elbow I Value", Constants.AutoConstants.elbowI);
        SmartDashboard.putNumber("Elbow D Value", Constants.AutoConstants.elbowD);

        //SmartDashboard.putNumber("Extension Setpoint", 0);
        SmartDashboard.putNumber("Extension P Value", Constants.AutoConstants.extensionP);
        SmartDashboard.putNumber("Extension I Value", Constants.AutoConstants.extensionI);
        SmartDashboard.putNumber("Extension D Value", Constants.AutoConstants.extensionD);

        //SmartDashboard.putNumber("Shoulder Setpoint", 0);
        SmartDashboard.putNumber("Shoulder P Value", Constants.AutoConstants.shoulderP);
        SmartDashboard.putNumber("Shoulder I Value", Constants.AutoConstants.shoulderI);
        SmartDashboard.putNumber("Shoulder D Value", Constants.AutoConstants.shoulderD);
        
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

    s_Arm.setSetpoints(wristSetpoint, shoulderSetpoint, elbowSetpoint, extensionSetpoint);

    //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
    //s_Arm.moveWrist(speed);
    s_Arm.wristPID();
    s_Arm.elbowPID();
    s_Arm.extensionPID();
    s_Arm.shoulderPID();
   }

   @Override
   public boolean isFinished () {
    //return pidController.atSetpoint();
    return s_Arm.isFinished();
   }

   @Override
   public void end (boolean interupted) {
    System.out.println("yep it sure did end before");
    s_Arm.moveElbow(0);
    s_Arm.moveShoulder(0);
    s_Arm.moveWrist(0);
    s_Arm.extendElbow(0);
    System.out.println("yep it sure did end...");
   }
}