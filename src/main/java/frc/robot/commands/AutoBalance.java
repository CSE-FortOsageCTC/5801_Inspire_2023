package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class AutoBalance extends CommandBase {
    boolean autoBalanceXMode;
    boolean autoBalanceYMode = true;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public double pitchAngleDegrees;
    private Swerve s_Swerve;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private double yAxisRate;
    private PIDController balancePID;

    public AutoBalance(frc.robot.subsystems.Swerve s_Swerve, boolean fieldRelative, boolean openLoop){
        this.s_Swerve = s_Swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.balancePID = new PIDController(0, 0, 0);
        this.balancePID.setSetpoint(0);
        this.balancePID.setTolerance(8);
        addRequirements(s_Swerve);  
        SmartDashboard.putNumber("BalP", 0);
        SmartDashboard.putNumber("BalI", 0);
        SmartDashboard.putNumber("BalD", 0);
    }

    @Override 
    public void execute(){        
        pitchAngleDegrees = s_Swerve.getRoll();
        //SmartDashboard.putBoolean("auto", true);
        //SmartDashboard.putNumber("Gyro Pitch", s_Swerve.getPitch());//displaying pitch value in degrees on dashboard
        //SmartDashboard.putNumber("yAxisRate", yAxisRate);
        //SmartDashboard.putNumber("AngleDegrees", pitchAngleDegrees);
        balancePID.setP(SmartDashboard.getNumber("BalP", 0.45));  
        balancePID.setI(SmartDashboard.getNumber("BalI", 0)); 
        balancePID.setD(SmartDashboard.getNumber("BalD", 0));

        double pitchAngleRadians = pitchAngleDegrees * (Math.PI / 180.0);
        yAxisRate = Math.sin(pitchAngleRadians) * -1;

        /*if (yAxisRate < 0.15 && yAxisRate > 0){
            yAxisRate = 0.15;
        } else if (yAxisRate > -0.15 && yAxisRate < 0){
            yAxisRate = -0.15;
        }*/
        SmartDashboard.putNumber("YAxisRate", yAxisRate);
        /*if ( autoBalanceYMode ) {
            
        }*/
        try {
            translation = new Translation2d(yAxisRate,0).times(Constants.Swerve.maxSpeed * 0.55);
            s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
        } catch( RuntimeException ex ) {
            String err_string = "Drive system error:  " + ex.getMessage();
            DriverStation.reportError(err_string, true);
        }
        //Timer.delay(0.1);	
    }
}