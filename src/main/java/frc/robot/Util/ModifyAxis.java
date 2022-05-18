// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Util;

import edu.wpi.first.math.MathUtil;

/** Add your docs here. */
public class ModifyAxis {
    public double m_value, m_deadValue, m_modifiedValue;
    /**
     * 
     * @param value The value that will be modified
     * @param exponent The power the value to be raised to
     */
    public ModifyAxis(double value, int exponent){
        m_value = value;
        m_deadValue = MathUtil.applyDeadband(value, 0.1);
        m_modifiedValue = Math.copySign(Math.pow(m_deadValue, exponent), m_deadValue);
    }

}
