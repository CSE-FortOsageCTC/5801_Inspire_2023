// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {
   
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  private final Joystick operator = new Joystick(1);
  
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int throttle = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton intakeInDriver = new JoystickButton(driver, XboxController.Axis.kLeftTrigger.value);
  private final JoystickButton intakeOutDriver = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton autoAlign = new JoystickButton(driver, XboxController.Button.kX.value);

  /* Operator Buttons */
  private final JoystickButton leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton intakeInOp = new JoystickButton(operator, XboxController.Axis.kRightTrigger.value);
  private final JoystickButton intakeOutOp = new JoystickButton(operator, XboxController.Axis.kLeftTrigger.value);

  /* D-Pad POV Driver */
  POVButton dpadUpDriver = new POVButton(driver, 0);
  POVButton dpadRightDriver = new POVButton(driver, 90);
  POVButton dpadDownDriver = new POVButton(driver, 180);
  POVButton dpadLeftDriver = new POVButton(driver, 270);
  
  /* D-Pad POV Operator */
  POVButton dpadUpOp = new POVButton(operator, 0);
  POVButton dpadRightOp = new POVButton(operator, 90);
  POVButton dpadDownOp = new POVButton(operator, 180);
  POVButton dpadLeftOp = new POVButton(operator, 270);

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final LEDSubsystem s_LEDSubsystem = new LEDSubsystem();
  private final IntakeSubsystem m_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();

  /* Variables */
  public static boolean isCone = true;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, throttle));
    s_ArmSubsystem.setDefaultCommand(new TeleopArm(s_ArmSubsystem, operator));

    // Configure the button bindings
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    leftBumper.onTrue(new InstantCommand(() -> setIsCone()));
    rightBumper.onTrue(new InstantCommand(() -> setIsCube()));
    intakeInDriver.whileTrue(new IntakeCommand(m_IntakeSubsystem, 0.3));
    intakeOutDriver.whileTrue(new IntakeCommand(m_IntakeSubsystem, -0.3));
    autoAlign.onTrue(new AutoAlign(s_Swerve, isCone));

    /* Operator Buttons */
    intakeInOp.whileTrue(new IntakeCommand(m_IntakeSubsystem, 0.3));
    intakeOutOp.whileTrue(new IntakeCommand(m_IntakeSubsystem, -0.3));

    /* D-Pad Driver Input Detection */
    dpadUpDriver.onTrue(dPadPOV(0));
    dpadRightDriver.onTrue(dPadPOV(90));
    dpadDownDriver.onTrue(dPadPOV(180));
    dpadLeftDriver.onTrue(dPadPOV(270));

    /* D-Pad Operator Input Detection */
    dpadUpOp.onTrue((dPadPOV(0)));
    dpadRightOp.onTrue((dPadPOV(90)));
    dpadDownOp.onTrue((dPadPOV(180)));
    dpadLeftOp.onTrue(dPadPOV(270));
  }

  public Command getAutonomousCommand() {
    return null; //new exampleAuto(s_Swerve);
  }

  /* Outputs D-Pad POV Value In Dashboard */
  public CommandBase dPadPOV (int angle) {

    System.out.println("D-Pad Angle is " + angle);
    SmartDashboard.putNumber("Angle", angle);
  
    return new RotateToHeading(s_Swerve, angle);

  }

  public void setIsCone() {

    isCone = true;

    //Sets LEDs to Yellow
    s_LEDSubsystem.SetLEDs(0.69);

    SmartDashboard.putBoolean("Cone Or Cube", isCone);
  }
  public void setIsCube() {

    isCone = false;

    //Sets LEDs to Purple
    s_LEDSubsystem.SetLEDs(0.91);

    SmartDashboard.putBoolean("Cone Or Cube", isCone);
  }
}
