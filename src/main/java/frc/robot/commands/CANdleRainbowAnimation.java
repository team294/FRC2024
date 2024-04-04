// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class CANdleRainbowAnimation extends Command {
  private LED led;
  private boolean fromShuffleboard;
  private LEDSegmentRange segment;

/** Creates a new CANdle Rainbow Animation
 * @param led led to use
 * @param segment segment to turn rainbow
 */
  public CANdleRainbowAnimation(LED led, LEDSegmentRange segment) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    fromShuffleboard = true;
    this.segment = segment;

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RainbowAnimation anim = new RainbowAnimation(1, .4, segment.count, false, segment.index);
    led.animate(anim);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    led.clearAnimation();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
