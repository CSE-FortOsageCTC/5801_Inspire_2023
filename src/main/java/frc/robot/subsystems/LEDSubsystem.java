package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  private double color;
  private final Spark blinkin;
  private final PowerDistribution PDP;

  public LEDSubsystem() {
    blinkin = new Spark(3);
    PDP = new PowerDistribution();
  }

  public void SetLEDs (double color) {
    this.color = color;
    blinkin.set(color);
  }

  public void setPrevious(){
    blinkin.set(color);
  }

  public void flashColor(double color){
    blinkin.set(color);
  }

  public void setSwitchableChannel(){
    PDP.setSwitchableChannel(false);
    PDP.setSwitchableChannel(true);
  }
}
