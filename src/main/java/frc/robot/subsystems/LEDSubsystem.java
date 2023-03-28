package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class LEDSubsystem extends SubsystemBase {

  private double color;
  private final Spark blinkin;

  public LEDSubsystem() {
    blinkin = new Spark(3);
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
}
