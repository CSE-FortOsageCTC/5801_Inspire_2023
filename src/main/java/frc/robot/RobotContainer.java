// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.Constants.AutoConstants.ArmPosition;
import frc.robot.autos.*;
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
  private final JoystickButton autoAlign = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton left18In = new JoystickButton(driver, XboxController.Button.kLeftBumper.value);
  private final JoystickButton right18In = new JoystickButton(driver, XboxController.Button.kRightBumper.value);
  private final JoystickButton autoBalance = new JoystickButton(driver, XboxController.Button.kB.value);

  /* Operator Buttons */
  private final JoystickButton leftBumper = new JoystickButton(operator, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rightBumper = new JoystickButton(operator, XboxController.Button.kRightBumper.value);
  private final JoystickButton xButton = new JoystickButton(operator, XboxController.Button.kX.value);
  private final JoystickButton yButton = new JoystickButton(operator, XboxController.Button.kY.value);
  private final JoystickButton aButton = new JoystickButton(operator, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(operator, XboxController.Button.kB.value);

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
  private final IntakeSubsystem s_IntakeSubsystem = new IntakeSubsystem();
  private final ArmSubsystem s_ArmSubsystem = new ArmSubsystem();

  /* Variables */
  public static boolean isCone = true;
  private SendableChooser<Command> m_autoChooser = new SendableChooser<>();

  private AutoAlign autoAlignCommand = new AutoAlign(s_Swerve, false);
  private AutoBalance autoBalanceCommand = new AutoBalance(s_Swerve, true, true);

  /* Paths */
  PathPlannerTrajectory Right18 = PathPlanner.loadPath("Right18inches", 1.5, 1.0);
  PathPlannerTrajectory Left18 = PathPlanner.loadPath("Left18inches", 1.5, 1.0);
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, throttle));
    s_ArmSubsystem.setDefaultCommand(new TeleopArm(s_ArmSubsystem, operator));
    s_IntakeSubsystem.setDefaultCommand(new IntakeCommand(s_IntakeSubsystem, operator));

    // Configure the button bindings
    configureButtonBindings();

    /* Auto Chooser Setup */
    m_autoChooser.setDefaultOption("A1B1A1C", new A1B1A1C(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.setDefaultOption("A1C", new A1C(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.addOption("A1B1A1", new A1B1A1(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.addOption("A1B1C", new A1B1C(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.addOption("A3B4A3", new A3B4A3(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.addOption("A3B4C", new A3B4C(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    m_autoChooser.addOption("testRotation", new TestRotate(s_Swerve, s_ArmSubsystem, s_IntakeSubsystem));
    SmartDashboard.putData("Auto Chooser", m_autoChooser);
  }

  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    autoAlign.whileTrue(autoAlignCommand);
    left18In.onTrue(s_Swerve.followTrajectoryCommand(Left18, true));
    right18In.onTrue(s_Swerve.followTrajectoryCommand(Right18, true));
    autoBalance.whileTrue(autoBalanceCommand);

    /* Operator Buttons */
    xButton.whileTrue(new PositionArm(s_ArmSubsystem, ArmPosition.Mid));
    yButton.whileTrue(new PositionArm(s_ArmSubsystem, ArmPosition.High));
    aButton.whileTrue(new PositionArm(s_ArmSubsystem, ArmPosition.Floor));
    bButton.whileTrue(new PositionArm(s_ArmSubsystem, ArmPosition.Default));
    leftBumper.onTrue(new InstantCommand(() -> setIsCone()));
    rightBumper.onTrue(new InstantCommand(() -> setIsCube()));

    /* D-Pad Driver Input Detection */
    dpadUpDriver.whileTrue(dPadPOV(0));
    dpadRightDriver.whileTrue(dPadPOV(90));
    dpadDownDriver.whileTrue(dPadPOV(180));
    dpadLeftDriver.whileTrue(dPadPOV(270));

    /* D-Pad Operator Input Detection */
    //dpadUpOp.whileTrue();
    //dpadRightOp.whileTrue();
    //dpadDownOp.whileTrue();
    dpadLeftOp.whileTrue(new PositionArm(s_ArmSubsystem, ArmPosition.Ramp));
  }

  public Command getAutonomousCommand() {
    return m_autoChooser.getSelected();
  }

  /* Outputs D-Pad POV Value In Dashboard */
  public CommandBase dPadPOV (int angle) {

    System.out.println("D-Pad Angle is " + angle);
    //SmartDashboard.putNumber("Angle", angle);
  
    return new RotateToHeading(s_Swerve, angle);

  }

  public void setIsCone() {

    isCone = true;

    //Sets LEDs to Yellow
    s_LEDSubsystem.SetLEDs(0.69);
    //autoAlignCommand.updateIsCone(isCone);
    SmartDashboard.putBoolean("Cone Or Cube", isCone);
  }
  public void setIsCube() {

    isCone = false;

    //Sets LEDs to Purple
    s_LEDSubsystem.SetLEDs(0.91);
    //autoAlignCommand.updateIsCone(isCone);
    SmartDashboard.putBoolean("Cone Or Cube", isCone);
  }

  /*public Command autoAlignCone() {
    return new AutoAlign(s_Swerve, true);
  }

  public Command autoAlignCube() {
    return new AutoAlign(s_Swerve, false);
  }*/
}
