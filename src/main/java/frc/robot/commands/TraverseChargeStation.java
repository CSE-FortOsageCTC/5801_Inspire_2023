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

public class TraverseChargeStation extends CommandBase {
    boolean autoBalanceXMode;
    boolean autoBalanceYMode = true;
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public PigeonIMU gyro;
    public double rollAngleDegrees;
    public double pitchAngleDegrees;
    private Swerve s_Swerve;
    private Translation2d translation;
    private boolean fieldRelative;
    private boolean openLoop;
    private boolean hasInclined = false;
    private boolean hasDeclined = false;
    private boolean hasLeveled = false;
    private double outputSpeed;

    public TraverseChargeStation(frc.robot.subsystems.Swerve s_Swerve, boolean fieldRelative, boolean openLoop, boolean hasInclined, boolean hasDeclined, boolean hasLeveled, double outputSpeed){
        addRequirements(s_Swerve);
        this.s_Swerve = s_Swerve;
        this.fieldRelative = fieldRelative;
        this.openLoop = openLoop;
        this.hasInclined = hasInclined; // Robot is starting to climb the Charge Station
        this.hasDeclined = hasDeclined; // Robot is passed the half way point on the Charge Station
        this.hasLeveled = hasLeveled; // Robot Roll is 0 after Traversing Charge Station
        this.outputSpeed = outputSpeed;

        gyro = new PigeonIMU(Constants.Swerve.pigeonID);
            gyro.configFactoryDefault();
    }

    public double getRoll() { // getting pitch
        rollAngleDegrees = gyro.getRoll();
        return rollAngleDegrees;

    }

    public double getPitch() { // getting pitch
        pitchAngleDegrees = gyro.getPitch();
        return pitchAngleDegrees;

    }
    
    @Override 
    public void execute(){    
        SmartDashboard.putNumber("Roll Degrees", getRoll());
        if (getRoll() > 5 || getPitch() > 5) {
            hasInclined = true;
        } else if (getRoll() < -5 || getPitch() < 5) {
            hasDeclined = true;
        } else if (getRoll() < 1 && getRoll() > -1  || getPitch() < 1 && getPitch() > 1 && hasInclined && hasDeclined) {
            hasLeveled = true;
        }

        translation = new Translation2d(-1, 0).times(Constants.Swerve.maxSpeed * 0.25);
                s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
        Timer.delay(0.005);
    }
        @Override
        public boolean isFinished () {
            return hasLeveled;
        }
    

    @Override
    public void end(boolean interrupted) {
        // TODO Auto-generated method stub
        translation = new Translation2d(0, 0);
                s_Swerve.drive(translation, 0.0, fieldRelative, openLoop);
        hasDeclined = false;
        hasInclined = false;
        hasLeveled = false;
    }
}