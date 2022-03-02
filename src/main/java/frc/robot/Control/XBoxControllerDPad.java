/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Control;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * One of the four arms of a {@link XBoxControllerEE.DPad} that gets its state from an {@link XBoxControllerEE}.
 */
public class XBoxControllerDPad extends Trigger
{

    private final XBoxControllerEE m_controller;
    private final int m_DPadValue;

    /**
     * Create a DPad object for triggering commands.
     *
     * @param ctrlr   The XboxController object that has that DPad
     * @param dpadArm The DPad arm
     */
    public XBoxControllerDPad(XBoxControllerEE ctrlr, XBoxControllerEE.DPad dpadArm)
    {
        m_controller = ctrlr;
        m_DPadValue = dpadArm.value;
    }

    /**
     * Gets the state of the DPad arm.
     *
     * @return The state of the DPad arm (true = Pressed; false = Unpressed)
     */
    public boolean get()
    {
        return (m_controller.getPOV(0) == m_DPadValue);
    }

	
}
