// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends CommandBase {
  // Set the ArmSubsystem as a variable, along with the variable speed located in ArmSubsystem
  private final ArmSubsystem m_ArmSubsystem;
  private final double speed;

  // Sets the parameters to their respected variables.
public ArmCommand(ArmSubsystem s_ArmSubsystem, double speed){
  this.m_ArmSubsystem = s_ArmSubsystem;
  this.speed = speed;
  addRequirements(s_ArmSubsystem);
}

// Moves arm, easy enough
@Override
public void execute(){
  m_ArmSubsystem.moveArm(speed);
}
}