// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeCommand extends CommandBase {
  private final IntakeSubsystem m_IntakeSubsystem;
  private Joystick controller;

public IntakeCommand(IntakeSubsystem m_IntakeSubsystem, Joystick controller){
  this.m_IntakeSubsystem = m_IntakeSubsystem;
  addRequirements(m_IntakeSubsystem);

  this.controller = controller;
}

@Override
public void execute(){
  double leftTriggerAxis = controller.getRawAxis(XboxController.Axis.kLeftTrigger.value);
  double rightTriggerAxis = controller.getRawAxis(XboxController.Axis.kRightTrigger.value);

  leftTriggerAxis = (Math.abs(leftTriggerAxis) < Constants.stickDeadband) ? 0 : leftTriggerAxis;
  rightTriggerAxis = (Math.abs(rightTriggerAxis) < Constants.stickDeadband) ? 0 : rightTriggerAxis;

  if (leftTriggerAxis > Constants.stickDeadband) {
    m_IntakeSubsystem.moveIntake(0.4);
  }
  else if (rightTriggerAxis > Constants.stickDeadband) {
    m_IntakeSubsystem.moveIntake(-0.5);
  }
  else {
    m_IntakeSubsystem.moveIntake(0);
  }
}
}