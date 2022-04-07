/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.Control;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * One of the four arms of a {@link DPad} that gets its state from an {@link XboxController}.
 */
public class XBoxControllerTrigger extends Trigger
{

    private final XBoxControllerEE m_controller;
    private final int m_axis;

    /**
     * Create an object treating the controller triggers as buttons.
     *
     * @param ctrlr   The XboxController object that has that DPad
     * @param dpadArm The DPad arm
     */
    public XBoxControllerTrigger(XBoxControllerEE ctrlr, XboxController.Axis trigger)
    {
        m_controller = ctrlr;
        m_axis = trigger.value;
    }

    /**
     * Gets the state of the specified Trigger axis.
     *
     * @return The state of the Trigger (true = Pressed (past 0.2); false = Unpressed (less than 0.2))
     */
    public boolean get()
    {
        return (m_controller.getRawAxis(m_axis) > 0.2);
    }

	
}
