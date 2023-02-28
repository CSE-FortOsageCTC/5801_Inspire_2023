package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.AnalogEncoder;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;

public class ArmSubsystem extends SubsystemBase {

    private final CANSparkMax armMasterMotor = new CANSparkMax(9, MotorType.kBrushless);
    private final CANSparkMax armSlaveMotor = new CANSparkMax(11, MotorType.kBrushless);
    private final TalonSRX elbowMotor = new TalonSRX(13);
    private final TalonSRX wristMotor = new TalonSRX(23);
    private final PIDController shoulderPID = new PIDController(0, 0, 0);
    private final PIDController elbowPID = new PIDController(0, 0, 0);
    private final PIDController wristPID = new PIDController(0, 0, 0);
    private final AnalogEncoder shoulderEncoder = new AnalogEncoder(0);
    private final AnalogEncoder elbowEncoder = new AnalogEncoder(0);
    private final AnalogEncoder wristEncoder = new AnalogEncoder(0);
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
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Shoulder Encoder Value", shoulderEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Elbow Encoder Value", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("Wrist Encoder Value", wristEncoder.getAbsolutePosition());
    }

    //Moves arm motors at a tenth of the input speed
    public CommandBase moveShoulder(double speed) {
        return run(()->armMasterMotor.set(speed / 10));
    }

    //Moves elbow motor
    public CommandBase moveElbow(double speed) {
        return run(()->elbowMotor.set(ControlMode.PercentOutput, speed));
    }

    //Moves wrist motor
    public CommandBase moveWrist(double speed) {
        return run(()->wristMotor.set(ControlMode.PercentOutput, speed));
    }

    /*
     * Moves shoulder motors to an input setoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public CommandBase shoulderPIDCommand(double setpoint/*, double feedbackSetpoint*/) {
        shoulderPID.setPID(SmartDashboard.getNumber("Shoulder P Value", 0), SmartDashboard.getNumber("Shoulder I Value", 0), SmartDashboard.getNumber("Shoulder D Value", 0));
        return run(()->armMasterMotor.set(shoulderPID.calculate(shoulderEncoder.getAbsolutePosition(), setpoint) /*+ shoulderFeedforward.calculate(feedbackSetpoint, setpoint)*/));
    }

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public CommandBase elbowPIDCommand(double setpoint/*, double feedbackSetpoint*/) {
        elbowPID.setPID(SmartDashboard.getNumber("Elbow P Value", 0), SmartDashboard.getNumber("Elbow I Value", 0), SmartDashboard.getNumber("Elbow D Value", 0));
        return run(()->elbowMotor.set(ControlMode.PercentOutput, elbowPID.calculate(elbowEncoder.getAbsolutePosition(), setpoint) /*+ elbowFeedforward.calculate(feedbackSetpoint, setpoint)*/));
    }

    /*
     * Moves elbow motor to an input setpoint with PID method
     * PID values determined from Smartdashboard for testing
     */
    public CommandBase wristPIDCommand(double setpoint/*, double feedbackSetpoint*/) {
        wristPID.setPID(SmartDashboard.getNumber("Wrist P Value", 0), SmartDashboard.getNumber("Wrist I Value", 0), SmartDashboard.getNumber("Wrist D Value", 0));
        return run(()->wristMotor.set(ControlMode.PercentOutput, wristPID.calculate(wristEncoder.getAbsolutePosition(), setpoint) /*+ wristFeedforward.calculate(feedbackSetpoint, setpoint)*/));
    }
}
