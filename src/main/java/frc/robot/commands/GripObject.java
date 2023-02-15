package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Gripper;

public class GripObject extends CommandBase{
    
    private boolean isCone;
    private boolean direction;
    private Gripper s_Gripper;

    public GripObject (Gripper s_Gripper, boolean isCone, boolean direction) {
        SmartDashboard.putString("Gripper Gripping?", "No");
        this.isCone = isCone;
        this.direction = direction;
        this.s_Gripper = s_Gripper;
    }
    @Override
    public void execute () {
        if (isCone) {

            if (direction) {
                //Run Cone Motors
            } else {
                //Run Cone Motors Opposite Direction
            }
            SmartDashboard.putString("Gripper Gripping?", "Gripping Cone");
        } else {
            if (direction) {
                //Run Cube Motors
            } else {
                //Run Cube Motors Opposite Direction
            }    
            SmartDashboard.putString("Gripper Gripping?", "Gripping Cube");
        }
        
    }

    @Override
    public void end(boolean interupted) {
        SmartDashboard.putString("Gripper Gripping?", "No");
    }


}
