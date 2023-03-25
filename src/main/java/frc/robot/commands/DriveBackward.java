// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** An example command that uses an example subsystem. */
public class DriveBackward extends CommandBase {
    private final Swerve m_SwerveSubsystem;

    public DriveBackward(Swerve m_SwerveSubsystem){
        this.m_SwerveSubsystem = m_SwerveSubsystem;
        addRequirements(m_SwerveSubsystem);
    }

    @Override
    public void execute(){
        m_SwerveSubsystem.drive(new Translation2d(-0.24, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
    }
}