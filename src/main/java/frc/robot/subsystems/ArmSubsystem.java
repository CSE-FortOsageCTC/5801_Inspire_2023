package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.EncoderType;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANDigitalInput.LimitSwitchPolarity;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax dartMotor = new CANSparkMax(9, MotorType.kBrushless);
    //private final CANSparkMax armSlaveMotor = new CANSparkMax(11, MotorType.kBrushless);
    //private final TalonSRX elbowMasterMotor = new TalonSRX(13);
    //private final VictorSPX elbowSlaveMotor = new VictorSPX(19);
    private final TalonSRX wristMotor = new TalonSRX(23);
    private final TalonSRX extensionMotor = new TalonSRX(22);

    private final PIDController dartPID = new PIDController(0, 0, 0);
    //private final PIDController elbowPID = new PIDController(0, 0, 0);
    private final PIDController wristPID = new PIDController(0, 0, 0);
    private final PIDController extensionPID = new PIDController(0, 0, 0);

    private final AnalogEncoder dartEncoder = new AnalogEncoder(0);
    //private final AnalogEncoder elbowEncoder = new AnalogEncoder(1);
    //private final AnalogEncoder wristEncoder = new AnalogEncoder(2);


    private final ArmFeedforward dartFeedforward = new ArmFeedforward(0, 0, 0);
    private final ArmFeedforward wristFeedforward = new ArmFeedforward(0, 0, 0);

    /*
     * Restors defaults for redundancy
     * Sets slave motor to inversely follow the master motor
     */
    public ArmSubsystem() {
        dartMotor.restoreFactoryDefaults();
        //armSlaveMotor.restoreFactoryDefaults();
        //armSlaveMotor.follow(armMasterMotor, true);
        
        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        extensionMotor.setNeutralMode(NeutralMode.Brake);

        /*
        elbowMasterMotor.setNeutralMode(NeutralMode.Brake);
        elbowMasterMotor.setInverted(true);
        elbowSlaveMotor.follow(elbowMasterMotor);
        elbowSlaveMotor.setNeutralMode(NeutralMode.Brake);
        elbowSlaveMotor.setInverted(true);
        */

        dartPID.setTolerance(0.005); //0.015
        wristPID.setTolerance(45); //0.05
        //elbowPID.setTolerance(0.015); //0.025
        extensionPID.setTolerance(1500); //1500


    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Dart Encoder Value", getDartEncoder());
        //SmartDashboard.putNumber("Elbow Encoder Value", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Encoder Value", getWristEncoder());
        SmartDashboard.putNumber("Extension Encoder Value", getExtensionEncoder());
        boolean extensionLimitSwitch = 1 == extensionMotor.isFwdLimitSwitchClosed();
        if (extensionLimitSwitch) {
            extensionMotor.setSelectedSensorPosition(0);
        }
        boolean dartLimitSwitch = dartMotor.getReverseLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen).isPressed();
        if (dartLimitSwitch) {
            //dartMotor.getEncoder().setPosition(0);
        }
        //SmartDashboard.putBoolean("Dart Limite Switch", dartLimitSwitch);
    }

    //Moves arm motors at a tenth of the input speed
    public void moveDart(double speed) {
       // if (getExtensionEncoder() < 0 /* CHANGE LATER */) {
        //    return;
        //}

        if (getDartEncoder() > Constants.AutoConstants.maxDartEncoder && speed > 0) {
            speed = 0;
        }
        if (getDartEncoder() > Constants.AutoConstants.maxDartEncoder - 3 && speed > 0) {
            speed = 0.1;
        }
        if (getDartEncoder() < 5 && speed < 0) {
            speed = -0.1;
        }
        if (getDartEncoder() < 0.5 && speed < 0) {
            speed = 0;
        }

        dartMotor.set(speed);
        SmartDashboard.putNumber("Dart Speed", speed);
    }

    /*
    //Moves elbow motor
    public void moveElbow(double speed) {
        elbowMasterMotor.set(ControlMode.PercentOutput, speed);
    }
    */

    //Moves wrist motor
    public void moveWrist(double speed) {
        SmartDashboard.putNumber("Wrist speed 2", speed);
        /*
        if (getElbowEncoder() < 0) { //TODO: change number to actual encoder position
            return;
        }*/
        if ((getWristEncoder() < Constants.AutoConstants.maxWristEncoder && speed < 0) || (getWristEncoder() > Constants.AutoConstants.minimumWristEncoder && speed > 0)) {
            wristMotor.set(ControlMode.PercentOutput, 0);
            return;
        }
        wristMotor.set(ControlMode.PercentOutput, speed);
    }

    //Moves elbow extension motor
    public void extendArm(double speed) {
        if (getDartEncoder() < 27.4 /* Change Later */) {
            
            return;
        }
        if (getExtensionEncoder() > Constants.AutoConstants.maxExtensionEncoder - Constants.AutoConstants.minExtensionEncoder && speed < 0) {
            speed = -0.12;
        }
        if (getExtensionEncoder() > Constants.AutoConstants.maxExtensionEncoder && speed < 0) {
            speed = 0;
        }
        if (getExtensionEncoder() < Constants.AutoConstants.minExtensionEncoder && speed > 0) {
            speed = 0.12;
        }
        extensionMotor.set(ControlMode.PercentOutput, speed);
    }

    /*
    public boolean feedForwardElbow(double position, boolean isPositive) {
        if (isPositive && position < getElbowEncoder()) {
            moveElbow(Constants.AutoConstants.maxElbowSpeed); //may need to multiply by -1
            return false;
        }
        if (!isPositive && position > getElbowEncoder()) {
            moveElbow(Constants.AutoConstants.maxElbowSpeed * -1); //may need to remove -1
            return false;
        }
        return true;
    }
    */

    public boolean feedForwardExtension(double position, boolean isPositive) {
        if (isPositive && position < getExtensionEncoder()) {
            extendArm(Constants.AutoConstants.maxExtensionSpeed); //may need to multiply by -1
            return false;
        }
        if (!isPositive && position > getExtensionEncoder()) {
            extendArm(Constants.AutoConstants.maxExtensionSpeed * -1); //may need to remove -1
            return false;
        }
        return true;
    }

    public boolean feedForwardWrist(double position, boolean isPositive) {
        if (isPositive && position < getWristEncoder()) {
            moveWrist(Constants.AutoConstants.maxWristSpeed); //may need to multiply by -1
            return false;
        }
        if (!isPositive && position > getWristEncoder()) {
            moveWrist(Constants.AutoConstants.maxWristSpeed * -1); //may need to remove -1
            return false;
        }
        return true;
    }

    public boolean feedForwardDart(double position, boolean isPositive) {
        System.out.println("FeedForwardDart Start");
        if (isPositive && position > getDartEncoder()) {
            moveDart(Constants.AutoConstants.maxDartSpeed); //may need to multiply by -1
            System.out.println("FeedForwardDart1");
            return false;
        }
        if (!isPositive && position < getDartEncoder()) {
            moveDart(-Constants.AutoConstants.maxDartSpeed); //may need to remove -1
            System.out.println("FeedForwardDart2");
            return false;
        }
        return true;
    }

    public double getDartEncoder() {
        return dartMotor.getEncoder().getPosition();
    }

    /*
    public double getElbowEncoder() {
        return elbowEncoder.getAbsolutePosition();
    }
    */

    public double getWristEncoder() {
        return wristMotor.getSelectedSensorPosition();
    }

    public double getExtensionEncoder() {
        return extensionMotor.getSelectedSensorPosition();
    }

    /*
     * Moves shoulder motors to an input setoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void dartPID(/*, double feedbackSetpoint*/) {
        dartPID.setPID(SmartDashboard.getNumber("Dart P Value", 0), SmartDashboard.getNumber("Dart I Value", 0), SmartDashboard.getNumber("Dart D Value", 0));
        //shoulderPID.setPID(Constants.AutoConstants.shoulderP, Constants.AutoConstants.shoulderI, Constants.AutoConstants.shoulderD);
        double speed = dartPID.calculate(getDartEncoder());
        speed = MathUtil.clamp(speed, -1, 1);
        dartMotor.set(speed/*+ shoulderFeedforward.calculate(feedbackSetpoint, setpoint)*/);
    }

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    /*
    public void elbowPID(/*, double feedbackSetpoint) {
        elbowPID.setPID(SmartDashboard.getNumber("Elbow P Value", 10), SmartDashboard.getNumber("Elbow I Value", 0), SmartDashboard.getNumber("Elbow D Value", 0));
        //elbowPID.setPID(Constants.AutoConstants.elbowP, Constants.AutoConstants.elbowI, Constants.AutoConstants.elbowD);
        double speed = elbowPID.calculate((elbowEncoder.getAbsolutePosition()));
        speed = MathUtil.clamp(speed, -0.6, 0.6);
        elbowMasterMotor.set(ControlMode.PercentOutput, speed) /*+ elbowFeedforward.calculate(feedbackSetpoint, setpoint);
        //SmartDashboard.putNumber("Elbow Speed", elbowPID.calculate(elbowEncoder.getAbsolutePosition()));
        //SmartDashboard.putString("Elbow Run", "It Ran");
    }
    */

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void wristPID() {
        wristPID.setPID(SmartDashboard.getNumber("Wrist P Value", 5.5), SmartDashboard.getNumber("Wrist I Value", 0), SmartDashboard.getNumber("Wrist D Value", 0));
        //wristPID.setPID(Constants.AutoConstants.wristP, Constants.AutoConstants.wristI, Constants.AutoConstants.wristD);
        double pidValue = wristPID.calculate(getWristEncoder());
        double speed = (pidValue);
        SmartDashboard.putNumber("Wrist Speed", speed);
        SmartDashboard.putNumber("PID Value", pidValue);
        speed = MathUtil.clamp(speed, -.75, .75);
        moveWrist(speed);
        SmartDashboard.putNumber("Wrist Setpoint", wristPID.getSetpoint());
    }

    /*
     * Moves elbow extension motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void extensionPID() {
        extensionPID.setPID(SmartDashboard.getNumber("Extension P Value", 0), SmartDashboard.getNumber("Extension I Value", 0), SmartDashboard.getNumber("Extension D Value", 0));
        //extensionPID.setPID(Constants.AutoConstants.extensionP, Constants.AutoConstants.extensionI, Constants.AutoConstants.extensionD);
        double speed = extensionPID.calculate(extensionMotor.getSelectedSensorPosition());
        speed = MathUtil.clamp(speed, -1, 1);
        extensionMotor.set(ControlMode.PercentOutput, -speed);
    }

    public void zeroExtensionEncoder() {
        extensionMotor.setSelectedSensorPosition(0);
    }

    public void outputArmValues() {

        System.out.println(String.valueOf(getDartEncoder()) + ", "/* + String.valueOf(getElbowEncoder()) + ", " */+ String.valueOf(getExtensionEncoder()) + ", " + (getWristEncoder() > 0.8 ? "maxWristEncoder" : "minimumWristEncoder"));    
    
    }

    public boolean isFinished() {
        //SmartDashboard.putBoolean("Did Extension PID?", extensionPID.atSetpoint());
        return dartPID.atSetpoint() /*&& elbowPID.atSetpoint() && wristPID.atSetpoint() */&& extensionPID.atSetpoint();
    }

    public void setSetpoints(double wrist, double dart, double extension) {
        wristPID.setSetpoint(wrist);
        dartPID.setSetpoint(dart);
        extensionPID.setSetpoint(extension);
    }

    public boolean isWristFinished() {
        return wristPID.atSetpoint();
    }

    public boolean isDartFinished() {
        return dartPID.atSetpoint();
    }

    public boolean isExtensionFinished() {
        return extensionPID.atSetpoint();
    }
}
