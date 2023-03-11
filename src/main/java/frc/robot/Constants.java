package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
        
    
    public static final double coneLimelightAreaSetpoint = .2;
    public static final double cubeLimelightAreaSetpoint = 10;
    public static final int conePipeline = 1;
    public static final int cubePipeline = 2;

    public static final class Swerve {
        public static final int pigeonID = 10;
        public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.5);
        public static final double wheelBase = Units.inchesToMeters(21.5);
        public static final double wheelDiameter = Units.inchesToMeters(3.94);
        public static final double wheelCircumference = wheelDiameter * Math.PI;

        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        public static final double driveGearRatio = (8.14 / 1.0); //6.86:1
        public static final double angleGearRatio = (12.8 / 1.0); //12.8:1

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Swerve Current Limiting */
        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 60;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* Angle Motor PID Values */
        public static final double angleKP = 0.6;
        public static final double angleKI = 0.0;
        public static final double angleKD = 12.0;
        public static final double angleKF = 0.0;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10;
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.0;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values */
        public static final double driveKS = (0.667 / 12); //divide by 12 to convert from volts to percent output for CTRE
        public static final double driveKV = (2.44 / 12);
        public static final double driveKA = (0.27 / 12);

        /* Swerve Profiling Values */
        public static final double maxSpeed = 4.5; //meters per second (default = 4.5)
        public static final double maxAngularVelocity = 6;    //8;   default is 11.5

        /* Neutral Modes */
        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        /* Motor Inverts */
        public static final boolean driveMotorInvert = true;
        public static final boolean angleMotorInvert = false;

        /* Angle Encoder Invert */
        public static final boolean canCoderInvert = false;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 {
            public static final int driveMotorID = 1;
            public static final int angleMotorID = 2;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(112.587);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 {
            public static final int driveMotorID = 3;
            public static final int angleMotorID = 4;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(188.349);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 {
            public static final int driveMotorID = 5;
            public static final int angleMotorID = 6;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(179.736);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(143.613);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

    }

    public static final class AutoConstants {
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;
    
        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        /*
         * Representation of the different positions the arm can be set to
         */
        public enum ArmPosition {
            Default(0.191, 0.625, -400, 0.276),
            Floor(0.270, 0.988, -60000, 0.761),
            Mid(0.307, 0.746, -4192, 0.603),
            High(0.320, 0.668, -68422, 0.622);
            /*
             * Representation of the motors that make up the arm
             */
            public enum ArmMotor {
                Shoulder,
                Elbow,
                Extension,
                Wrist;
            }
            /*
             * A map of the setpoints for the motors in the given arm position
             */
            private Map<ArmMotor, Double> setpoints;
            /**
             * Constructor for armposition enum
             * @param shoulderSetpoint Setpoint for the shoulder motor
             * @param elbowSetpoint Setpoint for the elbow motor
             * @param extensionSetpoint Setpoint for the shoulder extension motor
             * @param wristSetpoint Setpoint for the wrist motor
             */
            ArmPosition(double shoulderSetpoint, double elbowSetpoint, double extensionSetpoint, double wristSetpoint) {
                setpoints = new HashMap<>();
                setpoints.put(ArmMotor.Shoulder, shoulderSetpoint);
                setpoints.put(ArmMotor.Elbow, elbowSetpoint);
                setpoints.put(ArmMotor.Extension, extensionSetpoint);
                setpoints.put(ArmMotor.Wrist, wristSetpoint);
            }
            /**
             * Gets the setpoint for the given motor
             * @param armMotor Motor to get setpoint from
             * @return Setpoint
             * 
             */
            public double getSetpoint(ArmMotor armMotor) {
                return setpoints.get(armMotor);
            }
        }

        //PID constants for wrist motor
        public static final double wristP = 6;
        public static final double wristI = 0;
        public static final double wristD = 0;

        //PID constants for elbow motor
        public static final double elbowP = 4.5;
        public static final double elbowI = 0;
        public static final double elbowD = 0;

        //PID constants for elbow extension motor
        public static final double extensionP = 0.0005;
        public static final double extensionI = 0.0001;
        public static final double extensionD = 0;

        //PID constants for shoulder motor
        public static final double shoulderP = 3.5;
        public static final double shoulderI = 0;
        public static final double shoulderD = 0;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }

}
