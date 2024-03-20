// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class SetSpeakerMode extends InstantCommand {
  boolean speakerMode;
  BCRRobotState robotState;
  FileLog log;

  /**
   * Immediately sets the shooter's mode to the given mode.
   * This will update wherever the object is used.
   * @param speakerMode true = speaker mode, false = amp mode
   * @param robotState
   * @param log
   */
  public SetSpeakerMode(boolean speakerMode, BCRRobotState robotState, FileLog log) {
    this.speakerMode = speakerMode;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.setSpeakerMode(speakerMode);
    log.writeLog(true, "SetSpeakerMode", "SpeakerMode", speakerMode);
  }
}
