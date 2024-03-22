// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class updateStateLEDs extends Command {
  FileLog log;
  LED led;
  LEDSegmentRange segment;
  /** Creates a new updateStateLEDs. */
  public updateStateLEDs(LED led, FileLog log, LEDSegmentRange segment) {
    this.led = led;
    this.log = log;
    this.segment = segment;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.updateStateLEDs(segment);
  }


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
