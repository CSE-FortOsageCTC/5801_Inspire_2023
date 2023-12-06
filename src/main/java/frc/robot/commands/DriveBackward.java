// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/** An example command that uses an example subsystem. */

/** Tells the code that this is a command
 * RLB
 */
public class DriveBackward extends CommandBase {
    /** Introduces swerve subsystem
     * RLB
     */
    private final Swerve m_SwerveSubsystem;
    /** Constructor for DriveBackward
     * 
     * @param m_SwerveSubsystem Calls the swerve subsystem
     * 
     * RLB
     */

    public DriveBackward(Swerve m_SwerveSubsystem){
        this.m_SwerveSubsystem = m_SwerveSubsystem;
        addRequirements(m_SwerveSubsystem);
    }

    @Override
    public void execute(){
        m_SwerveSubsystem.drive(new Translation2d(-0.25, 0).times(Constants.Swerve.maxSpeed), 0, true, true);
    }
}