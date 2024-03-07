// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.concurrent.CancellationException;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utilities.FileLog;

public class LED extends SubsystemBase {
  private final FileLog log;
  private final CANdle candle;
  private String subsystemName;

  /** Creates a new LED. */
  public LED(int CANPort, String subsystemName, FileLog log) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
  }

  /** Gets the subsystem name */
  public String getName() {
    return subsystemName;
  }
  
  /** Clears all LEDs and animations */
  public void stop() {
    clearLEDs();
    clearAnimation();
  }
  
  /** Clears all LEDs */
  public void clearLEDs() {
    candle.setLEDs(0, 0, 0);
  }
  
  /** Clears all animations */
  public void clearAnimation() {
    candle.clearAnimation(0);
  }

  /**
   * Set the animation
   * @param anim the animation to set
  */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Set all the LEDs to a specific colour
   * @param r the red portion of the colour (0-255)
   * @param g the green portion of the colour (0-255)
   * @param b the blue portion of the colour (0-255)
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  /**
   * Sets a specific range of LEDs to a specific colour
   * @param r the red portion of the colour (0-255)
   * @param g the green portion of the colour (0-255)
   * @param b the blue portion of the colour (0-255)
   * @param w the white portion of the colour (0-255)
   * @param index the LED index to start the range at (first index is 0?)
   * @param count the number of LEDs in the range
   */
  public void setLEDs(int r, int g, int b, int w, int index, int count) {
    candle.setLEDs(r, g, b, w, index, count);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}