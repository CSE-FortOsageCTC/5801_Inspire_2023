package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.AutoRotateUtil;
import frc.robot.Constants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Swerve;

/**
 * This class detects the "april tag" and positions the robot infront of the node/April Tag
 */
public class AutoAlign extends CommandBase {

    private PIDController yTranslationPidController;
    private PIDController xTranslationPidController;
    private PIDController xTranslationConeController;
    private PIDController rotationPidController;

    private Swerve s_Swerve;
    private Limelight limelight;
    private AutoRotateUtil autoUtil;
    private boolean isCone;

    /**
     * Constructer for AutoAlign
     * 
     * @param s_Swerve swerve subsystem to be aligned with the april tag
     */
    public AutoAlign(Swerve s_Swerve, boolean isCone) {
        limelight = new Limelight(isCone);
        this.s_Swerve = s_Swerve;
        this.autoUtil = new AutoRotateUtil(s_Swerve, 180);
        addRequirements(s_Swerve);
        this.isCone = isCone;

        // creating yTranslationPidController and setting the toleance and setpoint
        yTranslationPidController = new PIDController(0, 0, 0);
        yTranslationPidController.setTolerance(1);
        yTranslationPidController.setSetpoint(0);
        
        // creating xTranslationPidController and setting the toleance and setpoint
        xTranslationPidController = new PIDController(0, 0, 0);
        //xTranslationPidController.setTolerance(isCone? .1 : 2);//5);
        xTranslationPidController.setSetpoint(isCone ? Constants.coneLimelightAreaSetpoint:Constants.cubeLimelightAreaSetpoint);
        
        xTranslationConeController = new PIDController(1.75, 0, 0.00035);
        xTranslationConeController.setSetpoint(Constants.coneLimelightAreaSetpoint);
        
        rotationPidController = new PIDController(0, 0, 0);
        rotationPidController.setTolerance(2);
        rotationPidController.setSetpoint(0);

        // puts the value of P,I and D onto the SmartDashboard
        // Will remove later
        SmartDashboard.putNumber("AlignP", 0.035);
        SmartDashboard.putNumber("AlignI", 0);
        SmartDashboard.putNumber("AlignD", 0.00035);

        //SmartDashboard.putNumber("RotationP", 0.035);
        //SmartDashboard.putNumber("RotationI", 0);
        //SmartDashboard.putNumber("RotationD", 0.00035);

    }

    @Override
    public void initialize() {
        limelight.updateIsCone(isCone);
        updateIsCone(isCone);
    }

    @Override
    public void execute() {
        // gets value of P,I and D from smartdashboard
        // will be removed
        double kP = SmartDashboard.getNumber("AlignP", 0);
        double kI = SmartDashboard.getNumber("AlignI", 0);
        double kD = SmartDashboard.getNumber("AlignD", 0);

        //double rotationkP = SmartDashboard.getNumber("RotationP", 0);
        //double rotationkI = SmartDashboard.getNumber("RotationI", 0);
        //double rotationkD = SmartDashboard.getNumber("RotationD", 0);
        
        // sets the PID values for the PIDControllers
        yTranslationPidController.setPID(kP, kI, kD);
        xTranslationPidController.setPID(kP, kI, kD);
        xTranslationPidController.setPID(kP, kI, kD);

        //rotationPidController.setPID(rotationkP, rotationkI, rotationkD);

        double xValue = limelight.getX(); //gets the limelight X Coordinate
        double areaValue = limelight.getArea(); // gets the area percentage from the limelight
        double angularValue = limelight.getSkew();
        
        SmartDashboard.putNumber("Limelightta", areaValue);

        SmartDashboard.putNumber("Xvalue", xValue);
        SmartDashboard.putNumber("Areavalue", areaValue);
        SmartDashboard.putNumber("AngularValue", angularValue);
        SmartDashboard.putNumber("AutoAlignSetpoint", xTranslationPidController.getSetpoint());
        //SmartDashboard.putNumber("Ts0", limelight.getSkew0());
        //SmartDashboard.putNumber("Ts1", limelight.getSkew1());
        //SmartDashboard.putNumber("Ts2", limelight.getSkew2());


        // Calculates the x and y speed values for the translation movement
        double ySpeed = MathUtil.clamp(yTranslationPidController.calculate(xValue), -0.5, 0.5);
        double xSpeed = MathUtil.clamp(isCone ? xTranslationConeController.calculate(areaValue):xTranslationPidController.calculate(areaValue), -0.5, 0.5);
        double angularSpeed =autoUtil.calculateRotationSpeed();//autoUtil.isFinished() ? 0: autoUtil.calculateRotationSpeed() * Constants.Swerve.maxAngularVelocity;
        
        SmartDashboard.putNumber("AlignXSpeed", xSpeed);
        SmartDashboard.putNumber("AlignYSpeed", ySpeed);

        // moves the swerve subsystem
        Translation2d translation = new Translation2d(-xSpeed, -ySpeed).times(Constants.Swerve.maxSpeed);
        double rotation = 0; //angularSpeed * Constants.Swerve.maxAngularVelocity;
        s_Swerve.drive(translation, rotation, true, true);

    }

    public void updateIsCone(boolean isCone) {
        this.isCone = isCone;
        xTranslationPidController.setSetpoint(isCone ? Constants.coneLimelightAreaSetpoint:Constants.cubeLimelightAreaSetpoint);
    }

    @Override
    public boolean isFinished() {

        //checks if the Swerve subsystem is within the given position tolerance
        SmartDashboard.putBoolean("AtSetPoint", yTranslationPidController.atSetpoint());
        return false; //yTranslationPidController.atSetpoint() && xTranslationPidController.atSetpoint(); // && autoUtil.isFinished();
    }

    @Override
    public void end(boolean end) {

        // tells the swerve subsystem to stop
        Translation2d translation = new Translation2d(0, 0).times(Constants.Swerve.maxSpeed);
        s_Swerve.drive(translation, 0, true, true);
        limelight.updateIsCone(false); // Switches pipeline to turn off Green LEDs
        xTranslationPidController.reset();
        yTranslationPidController.reset();
    }
}
