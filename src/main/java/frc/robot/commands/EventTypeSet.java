// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class EventTypeSet extends InstantCommand {
  BCRRobotState robotState;
  BCRRobotState.EventType eventType;
  FileLog log;

  /**
   * Immediately sets the Event Type
   * This will update wherever the object is used.
   * @param eventType event type to set (ex: COMPETITION)
   * @param robotState robotState object
   * @param log log
   */
  public EventTypeSet(BCRRobotState.EventType eventType, BCRRobotState robotState, FileLog log) {
    this.eventType = eventType;
    this.robotState = robotState;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    robotState.setEventType(eventType);
    log.writeLog(true, "EventTypeSet", eventType.toString());
  }
}
  