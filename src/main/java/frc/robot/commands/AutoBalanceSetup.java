package frc.robot.commands;

import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;
import frc.robot.subsystems.Swerve;

public class AutoBalanceSetup extends CommandBase {
    boolean autoBalanceXMode;
    boolean autoBalanceYMode = true;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public double pitchAngleDegrees;
    private Swerve s_Swerve;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;

    public AutoBalanceSetup(frc.robot.subsystems.Swerve s_Swerve, boolean fieldRelative, boolean openLoop){
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        
        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
            gyro.configFactoryDefault();
    }

    public boolean getPitch() { // getting pitch
        pitchAngleDegrees = gyro.getPitch();
        return (Constants.Swerve.invertGyro);

    }

    @Override 
    public void execute(){        
        getPitch();
        SmartDashboard.putBoolean("auto", true);//verifies if code actually runs
        SmartDashboard.putNumber("AngleDegrees", pitchAngleDegrees);////displaying pitch value in degrees on dashboard

        while (pitchAngleDegrees < 20) {

        }
        try {
            translation = new Translation2d(1, 0).times(Constants.Swerve.maxSpeed/1.5);
            s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
        } catch( RuntimeException ex ) {
            String err_string = "Drive system error:  " + ex.getMessage();
            DriverStation.reportError(err_string, true);
        }
        Timer.delay(0.005);	
    }
}