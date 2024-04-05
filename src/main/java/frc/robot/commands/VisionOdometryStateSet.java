// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class VisionOdometryStateSet extends InstantCommand {
  private final boolean enabled;
  private final DriveTrain driveTrain;
  private final FileLog log;

  /**
   * Immediately sets the Robot State object to the given state.
   * This will update wherever the object is used.
   * @param enabled 
   * @param driveTrain
   * @param log log
   */
  public VisionOdometryStateSet(boolean enabled, DriveTrain driveTrain, FileLog log) {
    this.enabled = enabled;
    this.driveTrain = driveTrain;
    this.log = log;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setVisionForOdomoetryState(enabled);
    log.writeLog(true, "VisionOdometryStateSet", (enabled) ? "1" : "0");
  }
}
  