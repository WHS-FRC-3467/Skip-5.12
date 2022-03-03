package frc.robot.Control;

/*----------------------------------------------------------------------------*/
/* Copyright (c) 2016-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID;


/**
 * Handle input from Xbox 360 or Xbox One controllers connected to the Driver Station.
 *
 * <p>
 * This class handles Xbox input that comes from the Driver Station. Each time a value is requested the most recent value is returned. There is a single class
 * instance for each controller and the mapping of ports to hardware buttons depends on the code in the Driver Station.
 */
public class XBoxControllerEE extends XboxController  
{

    /**
     * Represents the POV DPad on an XboxController.
     */
    public enum DPad
    {
        kDPadUp(0), //
        kDPadRight(90), //
        kDPadDown(180), //
        kDPadLeft(270); //

        public final int value;

        DPad(int value)
        {
            this.value = value;
        }
    }

    /**
     * Construct an instance of a joystick. The joystick index is the USB port on the drivers station.
     *
     * @param port The port on the Driver Station that the joystick is plugged into.
     */
    public XBoxControllerEE(final int port)
    {
        super(port);
    }

 
    /**
     * Whether the specified DPad direction is pressed
     *
     * @return Whether the direction button was pressed since the last check.
     */
    public boolean getDpadUp()
    {
        return (getPOV(0) == 0 ? true : false);
    }

    public boolean getDpadRight()
    {
        return (getPOV(0) == 90 ? true : false);
    }

    public boolean getDpadDown()
    {
        return (getPOV(0) == 180 ? true : false);
    }

    public boolean getDpadLeft()
    {
        return (getPOV(0) == 270 ? true : false);
    }


    public void setRumbleState(final boolean rumbleOn)
    {
        setRumble(GenericHID.RumbleType.kLeftRumble, rumbleOn ? 1 : 0);
        setRumble(GenericHID.RumbleType.kRightRumble, rumbleOn ? 1 : 0);
    }

    public void setRumbleValue(final double rumbleValue)
    {
        setRumble(GenericHID.RumbleType.kLeftRumble, rumbleValue);
        setRumble(GenericHID.RumbleType.kRightRumble, rumbleValue);
    }

}
