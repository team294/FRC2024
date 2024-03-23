// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class FarShotSet extends InstantCommand {
  boolean farShotMode;
  BCRRobotState robotState;
  FileLog log;

  /**
   * Immediately sets the shooter's mode to the given mode.
   * This will update wherever the object is used.
   * @param farShotMode true = farShotMode, false = amp mode
   * @param robotState
   * @param log
   */
  public FarShotSet(boolean farShotMode, BCRRobotState robotState, FileLog log) {
    this.farShotMode = farShotMode;
    this.robotState = robotState;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.setFarShotMode(farShotMode);
    log.writeLog(true, "SetFarShotMode", "farShotMode", farShotMode);
  }
}
