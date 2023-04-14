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
    private double dartSetpoint = 0;
    private double extensionSetpoint = 0;
    private double extensionTimer = 0;
    private boolean multiPosition = false;
    private int positionNumber = 0;
    private double lastWristSetpoint = 0;
    private double lastDartSetpoint = 0;
    private double lastExtensionSetpoint = 0;
    private List <ArmPosition> positions = new ArrayList<ArmPosition>();

    //private String m_position;
    public PositionArm(ArmSubsystem s_Arm, ArmPosition position) {
        this.s_Arm = s_Arm;
        this.position = position;

        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        dartSetpoint = position.getSetpoint(ArmMotor.Dart);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        
        SmartDashboard.putNumber("Wrist Setpoint", 0.48);
        SmartDashboard.putNumber("Wrist P Value", Constants.AutoConstants.wristP);
        SmartDashboard.putNumber("Wrist I Value", Constants.AutoConstants.wristI);
        SmartDashboard.putNumber("Wrist D Value", Constants.AutoConstants.wristD);

        //SmartDashboard.putNumber("Dart Setpoint", 0.75);
        SmartDashboard.putNumber("Dart P Value", Constants.AutoConstants.dartP);
        SmartDashboard.putNumber("Dart I Value", Constants.AutoConstants.dartI);
        SmartDashboard.putNumber("Dart D Value", Constants.AutoConstants.dartD);

        //SmartDashboard.putNumber("Extension Setpoint", 0);
        SmartDashboard.putNumber("Extension P Value", Constants.AutoConstants.extensionP);
        SmartDashboard.putNumber("Extension I Value", Constants.AutoConstants.extensionI);
        SmartDashboard.putNumber("Extension D Value", Constants.AutoConstants.extensionD);
        
    }

    public PositionArm(ArmSubsystem s_Arm, List<ArmPosition> positions) {
        this.positions = positions;
        this.multiPosition = true;
        this.s_Arm = s_Arm;

        ArmPosition position = positions.get(positions.size()-1);
        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
        dartSetpoint = position.getSetpoint(ArmMotor.Dart);
        extensionSetpoint = position.getSetpoint(ArmMotor.Extension);
        
        SmartDashboard.putNumber("Wrist Setpoint", 0.48);
        SmartDashboard.putNumber("Wrist P Value", Constants.AutoConstants.wristP);
        SmartDashboard.putNumber("Wrist I Value", Constants.AutoConstants.wristI);
        SmartDashboard.putNumber("Wrist D Value", Constants.AutoConstants.wristD);

        /*
        //SmartDashboard.putNumber("Elbow Setpoint", 0.75);
        SmartDashboard.putNumber("Elbow P Value", Constants.AutoConstants.elbowP);
        SmartDashboard.putNumber("Elbow I Value", Constants.AutoConstants.elbowI);
        SmartDashboard.putNumber("Elbow D Value", Constants.AutoConstants.elbowD);
        */

        //SmartDashboard.putNumber("Extension Setpoint", 0);
        SmartDashboard.putNumber("Extension P Value", Constants.AutoConstants.extensionP);
        SmartDashboard.putNumber("Extension I Value", Constants.AutoConstants.extensionI);
        SmartDashboard.putNumber("Extension D Value", Constants.AutoConstants.extensionD);
    }

   @Override
   public void initialize() {
    lastWristSetpoint = s_Arm.getWristEncoder();
    lastDartSetpoint = s_Arm.getDartEncoder();
    lastExtensionSetpoint = s_Arm.getExtensionEncoder();
   }

   @Override
   public void execute () {
    
    if (multiPosition && positionNumber != positions.size() - 1) {
        System.out.println("Setpoint " + positions.get(positionNumber).getSetpoint(ArmMotor.Dart));
        System.out.println("Last Setpoint " + lastDartSetpoint);
        System.out.println("Encoder " + s_Arm.getDartEncoder());
        if (/*s_Arm.feedForwardElbow(positions.get(positionNumber).getSetpoint(ArmMotor.Elbow),lastElbowSetpoint < s_Arm.getElbowEncoder()) && */
            s_Arm.feedForwardExtension(positions.get(positionNumber).getSetpoint(ArmMotor.Extension), lastExtensionSetpoint <= s_Arm.getExtensionEncoder()) &&
            //s_Arm.feedForwardWrist(positions.get(positionNumber).getSetpoint(ArmMotor.Wrist), lastWristSetpoint < s_Arm.getWristEncoder()) &&
            s_Arm.feedForwardDart(positions.get(positionNumber).getSetpoint(ArmMotor.Dart), lastDartSetpoint <= positions.get(positionNumber).getSetpoint(ArmMotor.Dart)))
            {
            System.out.println("Position Arm Multi");
            lastWristSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Wrist);
            lastDartSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Dart);
            lastExtensionSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Extension);
            positionNumber ++;
            s_Arm.moveDart(0);
            s_Arm.moveWrist(0);
            s_Arm.extendArm(0);
        }
        
    } else {
        s_Arm.moveDart(0);
        s_Arm.moveWrist(0);
        s_Arm.extendArm(0);
        if (wristSetpoint == Constants.AutoConstants.rotateWristNearest) {
            double halfwayPoint = ((Constants.AutoConstants.maxWristEncoder - Constants.AutoConstants.minimumWristEncoder) / 2) + Constants.AutoConstants.minimumWristEncoder;
            wristSetpoint = s_Arm.getWristEncoder() > halfwayPoint ? Constants.AutoConstants.maxWristEncoder : Constants.AutoConstants.minimumWristEncoder;
        }
        s_Arm.setSetpoints(wristSetpoint, dartSetpoint, extensionSetpoint);
    
        //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
        //s_Arm.moveWrist(speed);
        if (wristSetpoint != -1) {
        s_Arm.wristPID();
        }
        if (dartSetpoint != -1) {
        s_Arm.dartPID();
        }
        if (extensionSetpoint != -1) {
        s_Arm.extensionPID();
        }
    }
    
   }

   @Override
   public boolean isFinished () {
    boolean isWristFinished = true;
    //boolean isElbowFinished = true;
    boolean isExtensionFinished = true;
    //return pidController.atSetpoint();
    boolean isDartFinished = true;
    if (wristSetpoint != -1) {
        isWristFinished = s_Arm.isWristFinished();
    }
    if (dartSetpoint != -1) {
        isDartFinished = s_Arm.isDartFinished();
    }
    if (extensionSetpoint != -1) {
        isExtensionFinished = s_Arm.isExtensionFinished();
    }
    return isWristFinished && isDartFinished && isExtensionFinished;
   }

   @Override
   public void end (boolean interupted) {
    System.out.println("yep it sure did end before");
    //s_Arm.moveElbow(0);
    s_Arm.moveDart(0);
    s_Arm.moveWrist(0);
    s_Arm.extendArm(0);
    positionNumber = 0;
    System.out.println("yep it sure did end...");
   }
}