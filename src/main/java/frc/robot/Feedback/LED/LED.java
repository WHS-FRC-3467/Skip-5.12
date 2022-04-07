package frc.robot.Feedback.LED;

import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PWMConstants;

public class LED extends SubsystemBase {
  /** Creates a new LED. */
  PWM blinkin = new PWM(PWMConstants.Blinkin1);

  public LED() {  
    blinkin.setBounds(2.003, 1.50, 1.50, 1.50, 0.999);
    blinkin.setPeriodMultiplier(PWM.PeriodMultiplier.k1X);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void noBallLight(){
    //off 
    blinkin.setSpeed(0.0);
  }
  public void oneBallLight(){
    //Flashing Green
    blinkin.setSpeed(-0.05);
  }
  public void twoBallLight(){
    //Solid Green
    blinkin.setSpeed(0.93);
  }
}
