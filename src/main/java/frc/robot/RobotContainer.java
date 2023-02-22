// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;

import frc.robot.autos.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
   
  /* Controllers */
  private final Joystick driver = new Joystick(0);
  //private XboxController controller = new XboxController(0);
  
  /* Drive Controls */
  private final int translationAxis = XboxController.Axis.kLeftY.value;
  private final int strafeAxis = XboxController.Axis.kLeftX.value;
  private final int rotationAxis = XboxController.Axis.kRightX.value;
  private final int throttle = XboxController.Axis.kRightTrigger.value;

  /* Driver Buttons */
  private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
  private final JoystickButton xButton = new JoystickButton(driver, XboxController.Button.kX.value);
  private final JoystickButton gripButton = new JoystickButton(driver, XboxController.Button.kB.value);
  private final JoystickButton gripButtonReverse = new JoystickButton(driver, XboxController.Button.kRightStick.value);


  /* POV */
  POVButton dpadUp = new POVButton(driver, 0);
  POVButton dpadRight = new POVButton(driver, 90);
  POVButton dpadDown = new POVButton(driver, 180);
  POVButton dpadLeft = new POVButton(driver, 270);
  

  /* Subsystems */
  private final Swerve s_Swerve = new Swerve();
  private final Gripper s_Gripper = new Gripper();


  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    boolean fieldRelative = true;
    boolean openLoop = true;
    s_Swerve.setDefaultCommand(new TeleopSwerve(s_Swerve, driver, translationAxis, strafeAxis, rotationAxis, fieldRelative, openLoop, throttle));

    // Configure the button bindings
    configureButtonBindings();
  }


  /*public boolean get() {
    int dPadValue = dPad.getPOV();
    System.out.println("D-pad POV is " + dPadValue);
    return (dPadValue == dPadDirection.direction) || (dPadValue == (dPadDirection.direction + 45) % 360) || (dPadValue == (dPadDirection.direction + 315) % 360);
  }*/

  public static boolean isCone = true;


  // private final JoystickButton pieceMode = new JoystickButton(dPad, XboxController.Button.kX.value);



  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    /* Driver Buttons */
    zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));
    xButton.onTrue(new InstantCommand(() -> toggleGamePiece()));
    gripButton.whileTrue(new GripObject(s_Gripper, isCone, true));
    gripButtonReverse.whileTrue(new GripObject(s_Gripper, isCone, false));

    /* D-Pad Controller Input Detection */
    dpadUp.onTrue(new RotateToHeading(s_Swerve, 0));
    dpadRight.onTrue(new RotateToHeading(s_Swerve, 90));
    dpadDown.onTrue(new RotateToHeading(s_Swerve, 180));
    dpadLeft.onTrue(new RotateToHeading(s_Swerve, 270));

    dpadUp.onTrue(new InstantCommand(() -> dPadPOV(0)));
    dpadRight.onTrue(new InstantCommand(() -> dPadPOV(90)));
    dpadDown.onTrue(new InstantCommand(() -> dPadPOV(180)));
    dpadLeft.onTrue(new InstantCommand(() -> dPadPOV(270)));

    
  }




  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return new exampleAuto(s_Swerve);
  }

  /* Outputs D-Pad POV Value In Dashboard */
  public void dPadPOV (int angle) {

    System.out.println("D-Pad Angle is " + angle);
    SmartDashboard.putNumber("Angle", angle);


  }

  /* Flips isCone Boolean */
  public void toggleGamePiece() {

    isCone = !isCone;
    System.out.println("isCone is " + isCone);
    SmartDashboard.putBoolean("Cone Or Cube", isCone);

  }
}
