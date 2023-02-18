// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  // Makes CANSparkMax variable "armMotor" with the ID of 17 and the MotorType Brushless
  private final CANSparkMax armMotor = new CANSparkMax(17, MotorType.kBrushless);
  
  // Sets the speed of the motor to "speed" with the command "moveArm"
  public ArmSubsystem() {}
  public void moveArm(double speed){
    armMotor.set(speed);
  }
}