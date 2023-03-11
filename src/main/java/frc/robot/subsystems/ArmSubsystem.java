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
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax armMasterMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax armSlaveMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final TalonSRX elbowMasterMotor = new TalonSRX(13);
    private final VictorSPX elbowSlaveMotor = new VictorSPX(19);
    private final TalonSRX wristMotor = new TalonSRX(23);
    private final TalonSRX extensionMotor = new TalonSRX(22);

    private final PIDController shoulderPID = new PIDController(0, 0, 0);
    private final PIDController elbowPID = new PIDController(0, 0, 0);
    private final PIDController wristPID = new PIDController(0, 0, 0);
    private final PIDController extensionPID = new PIDController(0, 0, 0);

    private final AnalogEncoder shoulderEncoder = new AnalogEncoder(0);
    private final AnalogEncoder elbowEncoder = new AnalogEncoder(1);
    private final AnalogEncoder wristEncoder = new AnalogEncoder(2);


    private final ArmFeedforward shoulderFeedforward = new ArmFeedforward(0, 0, 0);
    private final ArmFeedforward elbowFeedforward = new ArmFeedforward(0, 0, 0);
    private final ArmFeedforward wristFeedforward = new ArmFeedforward(0, 0, 0);

    /*
     * Restors defaults for redundancy
     * Sets slave motor to inversely follow the master motor
     */
    public ArmSubsystem() {
        armMasterMotor.restoreFactoryDefaults();
        armSlaveMotor.restoreFactoryDefaults();
        armSlaveMotor.follow(armMasterMotor, true);
        
        extensionMotor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder);
        extensionMotor.setNeutralMode(NeutralMode.Brake);

        elbowMasterMotor.setNeutralMode(NeutralMode.Brake);
        elbowMasterMotor.setInverted(true);
        elbowSlaveMotor.follow(elbowMasterMotor);
        elbowSlaveMotor.setNeutralMode(NeutralMode.Brake);
        elbowSlaveMotor.setInverted(true);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Value", shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow Encoder Value", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Encoder Value", wristEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow Extension Encoder Value", extensionMotor.getSelectedSensorPosition());
        boolean limitSwitch = 1 == extensionMotor.isFwdLimitSwitchClosed();
        if (limitSwitch) {
            extensionMotor.setSelectedSensorPosition(0);
        }
    }

    //Moves arm motors at a tenth of the input speed
    public void moveShoulder(double speed) {
        armMasterMotor.set(speed);
    }

    //Moves elbow motor
    public void moveElbow(double speed) {
        elbowMasterMotor.set(ControlMode.PercentOutput, speed);
    }

    //Moves wrist motor
    public void moveWrist(double speed) {
        wristMotor.set(ControlMode.PercentOutput, speed);
    }

    //Moves elbow extension motor
    public void extendElbow(double speed) {
        extensionMotor.set(ControlMode.PercentOutput, speed);
    }

    public double getShoulderEncoder() {
        return shoulderEncoder.getAbsolutePosition();
    }

    public double getElbowEncoder() {
        return elbowEncoder.getAbsolutePosition();
    }

    public double getWristEncoder() {
        return wristEncoder.getAbsolutePosition();
    }

    public double getExtensionEncoder() {
        return extensionMotor.getSelectedSensorPosition();
    }

    /*
     * Moves shoulder motors to an input setoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void shoulderPID(double setpoint/*, double feedbackSetpoint*/) {
        shoulderPID.setPID(SmartDashboard.getNumber("Shoulder P Value", 0), SmartDashboard.getNumber("Shoulder I Value", 0), SmartDashboard.getNumber("Shoulder D Value", 0));
        //shoulderPID.setPID(Constants.AutoConstants.shoulderP, Constants.AutoConstants.shoulderI, Constants.AutoConstants.shoulderD);
        armMasterMotor.set((shoulderPID.calculate(shoulderEncoder.getAbsolutePosition(), setpoint)) * -1 /*+ shoulderFeedforward.calculate(feedbackSetpoint, setpoint)*/);
    }

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void elbowPID(double setpoint/*, double feedbackSetpoint*/) {
        elbowPID.setPID(SmartDashboard.getNumber("Elbow P Value", 10), SmartDashboard.getNumber("Elbow I Value", 0), SmartDashboard.getNumber("Elbow D Value", 0));
        //elbowPID.setPID(Constants.AutoConstants.elbowP, Constants.AutoConstants.elbowI, Constants.AutoConstants.elbowD);
        double speed = elbowPID.calculate(elbowEncoder.getAbsolutePosition(), setpoint);
        speed = MathUtil.clamp(speed, -0.5, 0.5);
        elbowMasterMotor.set(ControlMode.PercentOutput, speed) /*+ elbowFeedforward.calculate(feedbackSetpoint, setpoint)*/;
        SmartDashboard.putNumber("Elbow Speed", elbowPID.calculate(elbowEncoder.getAbsolutePosition(), setpoint));
        SmartDashboard.putString("Elbow Run", "It Ran");
    }

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void wristPID(double setpoint/*, double feedbackSetpoint*/) {
        wristPID.setPID(SmartDashboard.getNumber("Wrist P Value", 5.5), SmartDashboard.getNumber("Wrist I Value", 0), SmartDashboard.getNumber("Wrist D Value", 0));
        //wristPID.setPID(Constants.AutoConstants.wristP, Constants.AutoConstants.wristI, Constants.AutoConstants.wristD);
        wristMotor.set(ControlMode.PercentOutput, (wristPID.calculate(wristEncoder.getAbsolutePosition(), setpoint) * -1) /*+ wristFeedforward.calculate(feedbackSetpoint, setpoint)*/);
        SmartDashboard.putNumber("Wrist Speed", wristMotor.getMotorOutputPercent());
    }

    /*
     * Moves elbow extension motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public void extensionPID(double setpoint) {
        extensionPID.setPID(SmartDashboard.getNumber("Extension P Value", 0), SmartDashboard.getNumber("Extension I Value", 0), SmartDashboard.getNumber("Extension D Value", 0));
        //extensionPID.setPID(Constants.AutoConstants.extensionP, Constants.AutoConstants.extensionI, Constants.AutoConstants.extensionD);
        extensionMotor.set(ControlMode.PercentOutput, extensionPID.calculate(extensionMotor.getSelectedSensorPosition(), setpoint));
    }

    public void zeroExtensionEncoder() {
        extensionMotor.setSelectedSensorPosition(0);
    }
}
