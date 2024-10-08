/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.utilities;

/**
 * Add your docs here.
 */
public interface Loggable {
    /**
     * 
     * @param enabled true is enabled for every cycle; false follows normal logging cycles
     */
    public void enableFastLogging(boolean enabled);
}