package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class detects the "april tag" and positions the robot infront of the node/April Tag
 */
public class AutoAlign extends CommandBase {

    private PIDController yTranslationPidController;
    private PIDController xTranslationPidController;
    private PIDController rotationPidController;

    private Swerve s_Swerve;
    private Limelight limelight = new Limelight();

    /**
     * Constructer for AutoAlign
     * 
     * @param s_Swerve swerve subsystem to be aligned with the april tag
     */
    public AutoAlign(Swerve s_Swerve) {
        this.s_Swerve = s_Swerve;
        addRequirements(s_Swerve);

        // creating yTranslationPidController and setting the toleance and setpoint
        yTranslationPidController = new PIDController(0, 0, 0);
        yTranslationPidController.setTolerance(1);
        yTranslationPidController.setSetpoint(0);
        
        // creating xTranslationPidController and setting the toleance and setpoint
        xTranslationPidController = new PIDController(0, 0, 0);
        xTranslationPidController.setTolerance(5);
        xTranslationPidController.setSetpoint(10);
        
        
        rotationPidController = new PIDController(0, 0, 0);
        rotationPidController.setTolerance(2);
        rotationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        SmartDashboard.putNumber("P", 0.035);
        SmartDashboard.putNumber("i", 0);
        SmartDashboard.putNumber("d", 0.00035);

        SmartDashboard.putNumber("RotationP", 0.035);
        SmartDashboard.putNumber("RotationI", 0);
        SmartDashboard.putNumber("RotationD", 0.00035);

    }

    @Override
    public void execute() {
        // gets value of P,I and D from smartdashboard
        // will be removed
        double kP = SmartDashboard.getNumber("P", 0);
        double kI = SmartDashboard.getNumber("i", 0);
        double kD = SmartDashboard.getNumber("d", 0);

        double rotationkP = SmartDashboard.getNumber("RotationP", 0);
        double rotationkI = SmartDashboard.getNumber("RotationI", 0);
        double rotationkD = SmartDashboard.getNumber("RotationD", 0);


        
        // sets the PID values for the PIDControllers
        yTranslationPidController.setPID(kP, kI, kD);
        xTranslationPidController.setPID(kP, kI, kD);
        rotationPidController.setPID(rotationkP, rotationkI, rotationkD);

        double xValue = limelight.getX(); //gets the limelight X Coordinate
        double areaValue = limelight.getArea(); // gets the area percentage from the limelight
        double angularValue = limelight.getSkew();

        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);
        SmartDashboard.putNumber("AngularValue", angularValue);
        SmartDashboard.putNumber("Ts0", limelight.getSkew0());
        SmartDashboard.putNumber("Ts1", limelight.getSkew1());
        SmartDashboard.putNumber("Ts2", limelight.getSkew2());


        // Calculates the x and y speed values for the translation movement
        double ySpeed = yTranslationPidController.calculate(xValue);
        double xSpeed = xTranslationPidController.calculate(areaValue);
        double angularSpeed = rotationPidController.calculate(angularValue);

        // moves the swerve subsystem
        Translation2d translation = new Translation2d(xSpeed, ySpeed).times(Constants.Swerve.maxSpeed);
        double rotation = angularSpeed * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);

    }

    @Override
    public boolean isFinished() {

        //checks if the Swerve subsystem is within the given position tolerance
        SmartDashboard.putBoolean("AtSetPoint", yTranslationPidController.atSetpoint());
        return yTranslationPidController.atSetpoint() && xTranslationPidController.atSetpoint() && rotationPidController.atSetpoint();
    }

    @Override
    public void end(boolean end) {

        // tells the swerve subsystem to stop
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, 0, true, true);

        SmartDashboard.putString("Is Ended", "Yes");
    }
}
