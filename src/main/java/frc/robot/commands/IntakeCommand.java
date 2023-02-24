// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem m_ArmSubsystem;
  private final double speed;

public IntakeCommand(IntakeSubsystem m_ArmSubsystem, double speed){
  this.m_ArmSubsystem = m_ArmSubsystem;
  this.speed = speed;
  addRequirements(m_ArmSubsystem);
}

@Override
public void execute(){
  m_ArmSubsystem.moveArm(speed);
}
}