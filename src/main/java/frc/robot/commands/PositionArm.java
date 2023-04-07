package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.Constants.AutoConstants.ArmPosition.ArmMotor;
import frc.robot.subsystems.ArmSubsystem;

public class PositionArm extends CommandBase{
    
    private final ArmSubsystem s_Arm;
    private ArmPosition position;
    private double wristSetpoint = 0;
    private double elbowSetpoint = 0;
    private double extensionSetpoint = 0;
    private double shoulderSetpoint = 0;
    private double extensionTimer = 0;
    private boolean multiPosition = false;
    private int positionNumber = 0;
    private double lastWristSetpoint = 0;
    private double lastElbowSetpoint = 0;
    private double lastExtensionSetpoint = 0;
    private List <ArmPosition> positions = new ArrayList<ArmPosition>();

    //private String m_position;
    public PositionArm(ArmSubsystem s_Arm, ArmPosition position) {
        this.s_Arm = s_Arm;
        this.position = position;

        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        elbowSetpoint = position.getSetpoint(ArmMotor.Elbow);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        shoulderSetpoint = position.getSetpoint(ArmMotor.Shoulder);
        
        SmartDashboard.putNumber("Wrist Setpoint", 0.48);
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

    public PositionArm(ArmSubsystem s_Arm, List<ArmPosition> positions) {
        this.positions = positions;
        this.multiPosition = true;
        this.s_Arm = s_Arm;

        lastWristSetpoint = s_Arm.getWristEncoder();
        lastElbowSetpoint = s_Arm.getElbowEncoder();
        lastExtensionSetpoint = s_Arm.getExtensionEncoder();
        ArmPosition position = positions.get(positions.size()-1);
        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        elbowSetpoint = position.getSetpoint(ArmMotor.Elbow);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        shoulderSetpoint = position.getSetpoint(ArmMotor.Shoulder);
        
        SmartDashboard.putNumber("Wrist Setpoint", 0.48);
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
    
    if (multiPosition && positionNumber != positions.size() - 2) {
        if (s_Arm.feedForwardElbow(positions.get(positionNumber).getSetpoint(ArmMotor.Elbow), lastElbowSetpoint < s_Arm.getElbowEncoder()) && 
            s_Arm.feedForwardExtension(positions.get(positionNumber).getSetpoint(ArmMotor.Extension), lastExtensionSetpoint < s_Arm.getExtensionEncoder()) &&
            s_Arm.feedForwardWrist(positions.get(positionNumber).getSetpoint(ArmMotor.Wrist), lastWristSetpoint < s_Arm.getWristEncoder())) 
            {
            lastWristSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Elbow);
            lastElbowSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Extension);
            lastExtensionSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Wrist);
            positionNumber ++;
        }
    }
    //double wristSetpoint = SmartDashboard.getNumber("Wrist Setpoint", 0.48);
    //double elbowSetpoint = SmartDashboard.getNumber("Elbow Setpoint", 0.75);
    //double extensionSetpoint = SmartDashboard.getNumber("Extension Setpoint", 0);
    //double shoulderSetpoint = SmartDashboard.getNumber("Shoulder Setpoint", 0);
    if (wristSetpoint == Constants.AutoConstants.rotateWristNearest) {
        double halfwayPoint = ((Constants.AutoConstants.maxWristEncoder - Constants.AutoConstants.minimumWristEncoder) / 2) + Constants.AutoConstants.minimumWristEncoder;
        wristSetpoint = s_Arm.getWristEncoder() > halfwayPoint ? Constants.AutoConstants.maxWristEncoder : Constants.AutoConstants.minimumWristEncoder;
    }
    s_Arm.setSetpoints(wristSetpoint, shoulderSetpoint, elbowSetpoint, extensionSetpoint);

    //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
    //s_Arm.moveWrist(speed);
    if (wristSetpoint != -1) {
    s_Arm.wristPID();
    }
    if (elbowSetpoint != -1) {
    s_Arm.elbowPID();
    }
    if (extensionSetpoint != -1) {
    s_Arm.extensionPID();
    }
    if (shoulderSetpoint != -1) {
    s_Arm.shoulderPID();
    }
   }

   @Override
   public boolean isFinished () {
    boolean isWristFinished = true;
    boolean isElbowFinished = true;
    boolean isExtensionFinished = true;
    //return pidController.atSetpoint();
    if (wristSetpoint != -1) {
        isWristFinished = s_Arm.isWristFinished();
    }
    if (elbowSetpoint != -1) {
        isElbowFinished = s_Arm.isElbowFinished();
    }
    if (extensionSetpoint != -1) {
        isExtensionFinished = s_Arm.isExtensionFinished();
    }
    return isWristFinished && isElbowFinished && isExtensionFinished;
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