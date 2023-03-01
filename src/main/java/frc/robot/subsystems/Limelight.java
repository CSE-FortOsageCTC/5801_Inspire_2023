package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * this class retrieves limelight values from the networktable
 */
public class Limelight extends SubsystemBase {

    private NetworkTable table;
    private NetworkTableEntry tid, tv, tx, ty, ta, ts;
    public NetworkTableEntry ts0, ts1, ts2;

    /**
     * Constructs Limelight Class
     */
    public Limelight(boolean isCone) {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        table.getEntry("pipeline").setNumber(isCone ? Constants.conePipeline: Constants.cubePipeline);
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        ts = table.getEntry("ts");
        tv = table.getEntry("tv");
        tid = table.getEntry("tid");
        ts0 = table.getEntry("ts0");
        ts1 = table.getEntry("ts1");
        ts2 = table.getEntry("ts2");
    }
 
    /**
     * Retrieving the april tag ID value
     * @return April tag ID value
     */
    public int getAprilValue(){
        tid = table.getEntry("tid");
        return (int) tid.getDouble(0.0);
    }

    /**
     * Checks if the April tag is detected
     * @return <code>true</code> when april tag is detected, <code>false</code> otherwise
     */
    public boolean hasTag() {                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                           
        tv = table.getEntry("tv");
        return tv.getDouble(0.0) == 1.0;
    }

    /**
     * Retrieves Horizontal Offset From Crosshair To Target
     */
    public double getX() {
        tx = table.getEntry("tx");
        return tx.getDouble(0.0);
    }

    /**
     * Retrieves Vertical Offset From Crosshair To Target
     */
    public double getY() {
        ty = table.getEntry("ty");
        return ty.getDouble(0.0);
    }

    /**
     * retrieves Target Area (0% of image to 100% of image)
     */
    public double getArea() {
        ta = table.getEntry("ta");
        return ta.getDouble(0.0);
    }

    /**
     * Retrieves Skew or rotation (-90 degrees to 0 degrees)
     */
    public double getSkew() {
        ts = table.getEntry("ts");
        return ts.getDouble(0.0);
    }

    public double getSkew0() {
        ts0 = table.getEntry("ts0");
        return ts0.getDouble(0.0);
    }
    
    public double getSkew1() {
        ts1 = table.getEntry("ts1");
        return ts1.getDouble(0.0);
    }
    
    public double getSkew2() {
        ts2 = table.getEntry("ts2");
        return ts2.getDouble(0.0);
    }

    /**
     * retrieves limelight values and prints them onto the log and smartdashboard
     */
    public void outputValues(){
        table = NetworkTableInstance.getDefault().getTable("limelight");
        System.out.println(getAprilValue());
        System.out.println(hasTag());
        System.out.println(getX());
        System.out.println(getY());
        System.out.println(getSkew());
        System.out.println(getArea());
        SmartDashboard.putNumber("April Tag  Value", getAprilValue());
        SmartDashboard.putBoolean("has Tag  ", hasTag());
        SmartDashboard.putNumber("X value   ", getX());
        SmartDashboard.putNumber("Y value ", getY());
        SmartDashboard.putNumber("Skew Value ", getSkew());
        SmartDashboard.putNumber("Area Value ", getArea());
        
    }
    
}
