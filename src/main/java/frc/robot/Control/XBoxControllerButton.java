/*----------------------------------------------------------------------------*/
/* Copyright (c) 2008-2017 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Button;
/**
 * A {@link Button} that gets its state from an {@link XboxController}.
 */
public class XBoxControllerButton extends Button
{

    private final XBoxControllerEE m_joystick;
    private final int m_buttonNumber;

    /**
     * Create a joystick button for triggering commands.
     *
     * @param joystick The XboxController object that has that button
     * @param kb   The button number (see {@link Button})
     */
    public XBoxControllerButton(XBoxControllerEE joystick, XboxController.Button kb)
    {
        m_joystick = joystick;
        m_buttonNumber = kb.value;
    }



	/**
     * Gets the value of the joystick button.
     *
     * @return The value of the joystick button
     */
    public boolean get()
    {
        return m_joystick.getRawButton(m_buttonNumber);
    }
}
