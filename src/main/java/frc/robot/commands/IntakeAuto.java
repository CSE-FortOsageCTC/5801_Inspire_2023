// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

/** An example command that uses an example subsystem. */
public class IntakeAuto extends CommandBase {
    private final IntakeSubsystem m_IntakeSubsystem;
    private double speed;

    public IntakeAuto(IntakeSubsystem m_IntakeSubsystem, double speed){
        this.m_IntakeSubsystem = m_IntakeSubsystem;
        addRequirements(m_IntakeSubsystem);
        this.speed = speed;
    }

    @Override
    public void execute(){
        m_IntakeSubsystem.moveIntake(speed);
    }
}