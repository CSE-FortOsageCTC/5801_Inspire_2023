package frc.robot;

import java.util.HashMap;
import java.util.Map;

import org.opencv.core.Point;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;
    public static final double stickLeftDeadband = 0.12;
        
    
    public static final double coneLimelightAreaSetpoint = .27;
    public static final double cubeLimelightAreaSetpoint = 2.6;
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
        public static final double maxSpeed = 10; //meters per second (default = 4.5)
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
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(180.526);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 {
            public static final int driveMotorID = 7;
            public static final int angleMotorID = 8;
            public static final int canCoderID = 18;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(142.223);
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

        /* Intake Speeds */
        public static final double intakeInAutoConstant = 0.7;
        public static final double intakeInTeleopConstant = 0.6;
        
        public static final double intakeOutAutoConstant = -0.4;
        public static final double intakeOutTeleopConstant = -0.4;

        /*
         * Representation of the different positions the arm can be set to
         */
        public enum ArmPosition {
            Default(0.0, 0.0, -1), //0.085    0.625
            Travel(8.309545516967773, 0, -1),
            TravelSequence(-1, minExtensionEncoder, -1),
            Floor(20, 1906.0,-1),//Changed already           0.651                   0.4283
            FloorCube(28.285499572753906, 2518.0, -1),
            FloorSequence(30.856, 0, -1),
            Mid(111.59651184082031, 1443.0, -1), //0.360                  0.4283
            MidSequence(-1, 0.0, -1),
            AutoMidSequence1(111.59651184082031, 0.0, maxWristEncoder),
            AutoMidSequence(111.59651184082031, 0.0, -1),
            MidPlace(85, 1443.0, -1),
            High(132, 4119.0, -1), //0.502                 0.4283
            HighSequence(132, -1, maxWristEncoder),
            HighSequence1(50, -1, -1),
            HighPlace(106.73999786376953, 4119.0, -1),
            InverseFloor(0.905, -58297, -1),

            AutoWristRotate(-1, 0, maxWristEncoder),

            minimumWrist(-1, -1, minimumWristEncoder),
            maxWrist(-1, -1, maxWristEncoder);            
            //Ramp(0.349, 0.862, 0); //0.534
            /*
             * Representation of the motors that make up the arm
             */
            public enum ArmMotor {
                Dart,
                Extension,
                Wrist;
            }
            /*
             * A map of the setpoints for the motors in the given arm position
             */
            private Map<ArmMotor, Double> setpoints;
            /**
             * Constructor for armposition enum
             * @param dartSetpoint Setpoint for the dart motor
             * @param extensionSetpoint Setpoint for the shoulder extension motor
             */
            ArmPosition(double dartSetpoint, double extensionSetpoint, double wristSetpoint) {
                setpoints = new HashMap<>();
                setpoints.put(ArmMotor.Dart, dartSetpoint);
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
        public static final double wristP = .003;
        public static final double wristI = 0;
        public static final double wristD = 0;

        //PID constants for dart motor
        public static final double dartP = 0.025;//6;
        public static final double dartI = 0;
        public static final double dartD = 0;

        //PID constants for extension motor
        public static final double extensionP = 0.005;//0.0005;
        public static final double extensionI = 0;
        public static final double extensionD = 0.00005;

        //Wrist Encoder Limits
        public static final double maxWristEncoder = -1573;
        public static final double minimumWristEncoder = 0;
        public static final int rotateWristNearest = -1;

        //Dart Encoder Limits
        public static final double maxBumperDartEncoder = 118.9;
        public static final double minDartEncoder = 0.0;
        public static final double maxDartEncoder = 132.0;

        //Extension Encoder Limits
        public static final double maxExtensionEncoder = 4616.0;
        public static final double minExtensionEncoder = 350.0;

        //Motor Max Speeds
        public static final double maxDartSpeed = 1;
        public static final double maxExtensionSpeed = 1;
        public static final double maxWristSpeed = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
            new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
      }
    
      public static final class FieldConstants {

        //Node setpoints
        public static final Point n1 = new Point(1.87, 0.50);
        public static final Point n2 = new Point(1.87, 1.06);
        public static final Point n3 = new Point(1.87, 1.59);
        public static final Point n4 = new Point(1.87, 2.18);
        public static final Point n5 = new Point(1.87, 2.73);
        public static final Point n6 = new Point(1.87, 3.32);
        public static final Point n7 = new Point(1.87, 3.86);
        public static final Point n8 = new Point(1.87, 4.44);
        public static final Point n9 = new Point(1.87, 5.00);

        //Game piece setpoints
        public static final Point g1 = new Point(7.07, 0.92);
        public static final Point g2 = new Point(7.07, 2.15);
        public static final Point g3 = new Point(7.07, 3.39);
        public static final Point g4 = new Point(7.07, 4.60);
        public static final Point highXClimb = new Point(2.36, 3.41);
        public static final Point lowXClimb = new Point(2.36, 2.13);

        //Charge station setpoints
        public static final Point lowXLowY = new Point(2.90, 1.52);
        public static final Point highXLowY = new Point(4.88, 1.52);
        public static final Point lowXHighY = new Point(2.90, 4.00);
        public static final Point highXHighY = new Point(4.88, 4.00);

        //Traversal setpoints
        public static final Point lowXLowYTraversal = new Point(2.90 + 0.629, 1.52 + 0.629);
        public static final Point highXLowYTraversal = new Point(4.88 + 0.629, 1.52 + 0.629);
        public static final Point lowXHighYTraversal = new Point(2.90 + 0.629, 4.00 + 0.629);
        public static final Point highXHighYTraversal = new Point(4.88 + 0.629, 4.00 + 0.629);
      }

}
