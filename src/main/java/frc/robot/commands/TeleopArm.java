package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TeleopArm extends CommandBase {
    
    private ArmSubsystem s_Arm;
    private Joystick controller;

    /**
     * Driver control
     */
    public TeleopArm(ArmSubsystem s_Arm, Joystick controller) {
        this.s_Arm = s_Arm;
        addRequirements(s_Arm);

        this.controller = controller;
    }

    @Override
    public void execute() {
        double leftYAxis = controller.getRawAxis(XboxController.Axis.kLeftY.value);
        double leftXAxis = controller.getRawAxis(XboxController.Axis.kLeftX.value);
        double rightYAxis = controller.getRawAxis(XboxController.Axis.kRightY.value);
        double rightXAxis = controller.getRawAxis(XboxController.Axis.kRightX.value);
        
        /* Deadbands */
        leftYAxis = (Math.abs(leftYAxis) < Constants.stickDeadband) ? 0 : leftYAxis;
        leftXAxis = (Math.abs(leftXAxis) < Constants.stickDeadband) ? 0 : leftXAxis;
        rightYAxis = (Math.abs(rightYAxis) < Constants.stickDeadband) ? 0 : rightYAxis;
        rightXAxis = (Math.abs(rightXAxis) < Constants.stickDeadband) ? 0 : rightXAxis;

        s_Arm.moveShoulder(leftYAxis * 0.3);
        s_Arm.extendElbow(leftXAxis * 0.6);
        s_Arm.moveElbow(rightYAxis * 0.6);
        s_Arm.moveWrist(rightXAxis * 0.8);
    }
}