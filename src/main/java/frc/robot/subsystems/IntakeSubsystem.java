// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANSparkMax intakeMasterMotor = new CANSparkMax(17, MotorType.kBrushless);
  //private final CANSparkMax intakeSlaveMotor = new CANSparkMax(24, MotorType.kBrushless);
  private LEDSubsystem ledSubsystem;

  public IntakeSubsystem(LEDSubsystem ledSubsystem) {
    this.ledSubsystem = ledSubsystem;
    intakeMasterMotor.restoreFactoryDefaults();
    //intakeSlaveMotor.restoreFactoryDefaults();
    intakeMasterMotor.setSmartCurrentLimit(20);
    intakeMasterMotor.burnFlash();
    //intakeSlaveMotor.follow(intakeMasterMotor, true);
  }

  public void moveIntake(double speed){
    SmartDashboard.putNumber("Current", intakeMasterMotor.getOutputCurrent()); 
    if (intakeMasterMotor.getOutputCurrent() > 15
    ){
    ledSubsystem.flashColor(.79);
    //return;
   }
   else if (speed < 0 ){
    ledSubsystem.flashColor(.59
    );
   }
   else{
    ledSubsystem.setPrevious();
   }
    intakeMasterMotor.set(speed);
  }
}