// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.LEDSegment;


public class LED extends SubsystemBase {
  private final FileLog log;
  private final CANdle candle;
  private String subsystemName;
  private BCRRobotState.State currentState;

  // LED Segments
  //private LEDSegment CANdle;
  //private LEDSegment strip1;
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  /**
   * Creates the CANdle LED subsystem.
   * @param subsystemName
   * @param log
   */
  public LED(int CANPort, String subsystemName, FileLog log) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.currentState = BCRRobotState.State.IDLE_NO_PIECE;

    // Create the LED segments
    LEDSegment CANdle = new LEDSegment(
      LEDConstants.LEDSegmentRange.CANdle.index,
      LEDConstants.LEDSegmentRange.CANdle.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment strip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip1.index,
      LEDConstants.LEDSegmentRange.Strip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    segments.put(LEDSegmentRange.CANdle, CANdle);
    segments.put(LEDSegmentRange.Strip1, strip1);
  }

  /** Get the subsystem's name
   * @return the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }
  
  /** Clear all LEDs */
  public void clearLEDs() {
    setLEDs(0, 0, 0);
  }

  /**
   * Clear the LEDs of a specific segment range
   * @param segment the segment to clear
   */
  public void clearLEDs(LEDSegmentRange segment) {
    setLEDs(0, 0, 0, segment);
  }
  
  /**
   * Clear all animation
   */
  public void clearAnimation() {
    candle.clearAnimation(0);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      setAnimation(LEDConstants.Patterns.noPatternAnimation, segmentKey, false);
    }
  }
  
  /**
   * Start a built-in animation
   * @param anim
   */
  public void animate(Animation anim) {
    candle.animate(anim);
  }

  /**
   * Set the pattern and resizes it to fit the LED strip
   * @param pattern the pattern to use
   * @param segment the segment to use
   */
  public void setPattern(Color[] pattern, LEDSegmentRange segment) {
    if (pattern.length == 0) return;
    for (int indexLED = 0, indexPattern = 0; indexLED < segment.count; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
  }

  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    if (segments.containsKey(segment)) {
      segments.get(segment).setAnimation(animation, loop);
    } else if (segment == LEDSegmentRange.Full) {
      // Special case
      segments.get(LEDSegmentRange.CANdle).setAnimation(animation, loop);
      segments.get(LEDSegmentRange.Strip1).setAnimation(animation, loop);
    }
  }

  /**
   * Sets LEDs using only R, G, and B
   * @param r red value
   * @param g green value
   * @param b blue value
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
  }

  /**
   * Takes in color and sets correct RGB values
   * @param color color to set
   */
  public void setLEDs(Color color) {
    setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
  }

  /**
   * Sets LEDs using RGB, index, and count values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(int r, int g, int b, int index, int count) {
    candle.setLEDs(r, g, b, 0, index, count);
  }
  /**
   * Sets LEDs using color and index values
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
  }
  /**
   * Sets LEDs using color, index, and count values
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, count);
  }
  /**
   * Sets LEDs using RGB and segment values
   * @param r red value
   * @param g green value
   * @param b blue value
   * @param segment segment to light up (range)
   */
  public void setLEDs(int r, int g, int b, LEDSegmentRange segment) {
    candle.setLEDs(r, g, b, 0, segment.index, segment.count);
  }
  /**
   * Sets LEDs using BCR color enum (ex: IDLE_NO_PIECE)
   * @param color color to set
   */
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.b);
  }
  
  /**
   * Sets LEDs using robot state (ex: IDLE_NO_PIECE)
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
  }

  public void updateStateAll(BCRRobotState.State state) {
    // Store the state for periodics
    currentState = state;
    // Update the LEDs now
    updateStateLEDs(LEDSegmentRange.Full, state);
  }

  public void updateStateLEDs(LEDSegmentRange segment, BCRRobotState.State state) {
    // Store the state for periodics
    currentState = state;
    // Set LEDs to match the state, as defined in Constants.BCRColor
    switch (state) {
    case IDLE_NO_PIECE:
      setLEDs(BCRColor.IDLE_NO_PIECE, segment.index, segment.count);
      break;
    case IDLE_WITH_PIECE:
      setLEDs(BCRColor.IDLE_WITH_PIECE, segment.index, segment.count);
      break;
    case INTAKE_NO_PIECE:
      setLEDs(BCRColor.INTAKE_NO_PIECE, segment.index, segment.count);
      break;
    case SHOOT_READY:
      setLEDs(BCRColor.SHOOT_READY, segment.index, segment.count);
      break;
    case STICKY_FAULTS:
      setLEDs(BCRColor.STICKY_FAULTS, segment.index, segment.count);
      break;
    }
  }

  @Override
  public void periodic() {
    // Every scheduler run, update the animations for all segments
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      // Display this segment
      setPattern(segments.get(segmentKey).getCurrentFrame(), segmentKey);
      // Move to the next frame
      boolean shouldClear = segments.get(segmentKey).advanceFrame();
      if (shouldClear) {
        updateStateLEDs(segmentKey, currentState);
      }
    }
  }
}