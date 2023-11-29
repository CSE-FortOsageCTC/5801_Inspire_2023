package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.MathUsageId;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class AutoBalSetupInvert extends CommandBase {
    boolean autoBalanceXMode;
    boolean autoBalanceYMode = true;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public double rollAngleDegrees;
    private Swerve s_Swerve;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    public AutoBalSetupInvert(frc.robot.subsystems.Swerve s_Swerve, boolean fieldRelative, boolean openLoop){
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
            gyro.configFactoryDefault();
    }

    public double getRoll() { // getting pitch
        rollAngleDegrees = gyro.getRoll();
        return rollAngleDegrees;

    }
    
    @Override 
    public void execute(){        
        
        SmartDashboard.putBoolean("auto", true); //displaying pitch value in degrees on dashboard
        translation = new Translation2d(1, 0).times(Constants.Swerve.maxSpeed * 0.25);
                s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
        Timer.delay(0.005);
    }

    /*@Override
    public boolean isFinished() {
        return Math.abs(getRoll()) > 5;
    }*/

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        translation = new Translation2d(0, 0);
                s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
    }
}