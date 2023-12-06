// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeSubsystem extends SubsystemBase {
  
  private final CANSparkMax intakeMasterMotor = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax intakeSlaveMotor = new CANSparkMax(24, MotorType.kBrushless);
  
  public IntakeSubsystem() {
    intakeMasterMotor.restoreFactoryDefaults();
    intakeSlaveMotor.restoreFactoryDefaults();
    intakeSlaveMotor.follow(intakeMasterMotor, true);
  }

  public void moveIntake(double speed){
    intakeMasterMotor.set(speed);
  }
}