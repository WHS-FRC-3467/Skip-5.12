package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Feedback.LED.BlinkinLedDriver.BlinkinLedMode;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  private final BlinkinLedDriver blinkin;

  public LED() {
    blinkin = new BlinkinLedDriver(0);
  }

  
  @Override
  public void periodic() {}

  public void noBallLight(){
    //off 
    blinkin.setMode(BlinkinLedMode.SOLID_BLACK);
  }
  public void twoBallLight(){
    //Flashing Green
    blinkin.setMode(BlinkinLedMode.FIXED_STROBE_GOLD);
  }
  public void oneBallLight(){
    //Solid Green
    blinkin.setMode(BlinkinLedMode.SOLID_GOLD);
  }
}
