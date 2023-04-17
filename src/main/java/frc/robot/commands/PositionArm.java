package frc.robot.commands;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
    private double dartFeedForwardSpeed = .6;

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

        System.out.println("ArmPosition List: " + positions);

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
    public PositionArm withDartSpeed(double speed){
        dartFeedForwardSpeed = speed;
        return this;
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
        //System.out.println("Setpoint " + positions.get(positionNumber).getSetpoint(ArmMotor.Extension));
        //System.out.println("Last Setpoint " + lastExtensionSetpoint);
        //System.out.println("Encoder " + s_Arm.getExtensionEncoder());
        boolean extensionFinished = true;
        boolean dartFinished = true;
        boolean isWristFinished = true;
        if  (positions.get(positionNumber).getSetpoint(ArmMotor.Wrist) != -1){
            s_Arm.setSetpoints(positions.get(positionNumber).getSetpoint(ArmMotor.Wrist) , dartSetpoint, extensionSetpoint);
            s_Arm.wristPID();
            isWristFinished = s_Arm.isWristFinished();
        }
        if (positions.get(positionNumber).getSetpoint(ArmMotor.Extension) != -1){
            extensionFinished = s_Arm.feedForwardExtension(positions.get(positionNumber).getSetpoint(ArmMotor.Extension), lastExtensionSetpoint <= positions.get(positionNumber).getSetpoint(ArmMotor.Extension));
        }
        if (positions.get(positionNumber).getSetpoint(ArmMotor.Dart) != -1){
            dartFinished = s_Arm.feedForwardDart(positions.get(positionNumber).getSetpoint(ArmMotor.Dart), lastDartSetpoint <= positions.get(positionNumber).getSetpoint(ArmMotor.Dart), dartFeedForwardSpeed);
        }
        if (extensionFinished && dartFinished && isWristFinished) {
            lastWristSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Wrist);
            lastDartSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Dart);
            lastExtensionSetpoint = positions.get(positionNumber).getSetpoint(ArmMotor.Extension);
            positionNumber ++;
            //s_Arm.moveDart(0);
            //s_Arm.moveWrist(0);
            //s_Arm.extendArm(0);
        }
        
    } else {
        //System.out.println("just something so we know we got there");
        s_Arm.moveDart(0);
        s_Arm.moveWrist(0);
        s_Arm.extendArm(0);
        if (wristSetpoint == Constants.AutoConstants.rotateWristNearest) {
            double halfwayPoint = ((Constants.AutoConstants.maxWristEncoder - Constants.AutoConstants.minimumWristEncoder) / 2) + Constants.AutoConstants.minimumWristEncoder;
            System.out.println(halfwayPoint);
            wristSetpoint = (s_Arm.getWristEncoder() < halfwayPoint )? Constants.AutoConstants.maxWristEncoder : Constants.AutoConstants.minimumWristEncoder;
        }
        s_Arm.setSetpoints(wristSetpoint, dartSetpoint, extensionSetpoint);
        /*
        if (s_Arm.getDartEncoder() >= 100) {
            s_Arm.moveDart(Constants.AutoConstants.maxDartSpeed);
            if (s_Arm.getExtensionEncoder() > 0) {
                s_Arm.extendArm(Constants.AutoConstants.maxExtensionSpeed * -1);
            }
        }
        */
        //double speed = pidController.calculate(s_Arm.getWristEncoder(), setpoint) * -1;
        //s_Arm.moveWrist(speed);
        //if (s_Arm.getDartEncoder() < 100) {
            if (wristSetpoint != -1) {
            s_Arm.wristPID();
            }
            if (dartSetpoint != -1) {
            s_Arm.dartPID();
            }
            if (extensionSetpoint != -1) {
            s_Arm.extensionPID();
            }
        //}
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
        if (!s_Arm.canWristMove() && !isWristFinished) {
            return true;
        }
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
    s_Arm.reset();
    if (multiPosition){
        ArmPosition position = positions.get(positions.size()-1);
        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
    }
    else{
        wristSetpoint = position.getSetpoint(ArmMotor.Wrist);
    }

    System.out.println("yep it sure did end...");
   }
}