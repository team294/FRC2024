// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.LEDConstants.*;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.LEDSegment;
import frc.robot.utilities.RobotPreferences;


public class LED extends SubsystemBase {
  private final FileLog log;
  private final CANdle candle;
  private String subsystemName;
  private BCRRobotState robotState;
  private BCRRobotState.State currentState;
  private Feeder feeder;
  private boolean stickyFault;
  private boolean shouldClear;
  private double accuracyDisplayThreshold;
  private int accuracy;

  private Color[] accuracyDisplayPattern = {Color.kRed, Color.kRed};

  // LED Segments
  //private LEDSegment CANdle;
  //private LEDSegment strip1;
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  /**
   * Creates the CANdle LED subsystem.
   * @param subsystemName
   * @param log
   */
  public LED(int CANPort, String subsystemName, BCRRobotState robotState, FileLog log, Feeder feeder) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.feeder = feeder;
    this.stickyFault = false;
    this.shouldClear = false;
    this.accuracyDisplayThreshold = 35;
    this.accuracy = 0;

    // Create the LED segments
    LEDSegment CANdleTop = new LEDSegment(
      LEDConstants.LEDSegmentRange.CANdleTop.index,
      LEDConstants.LEDSegmentRange.CANdleTop.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment CANdleBottom = new LEDSegment(
      LEDConstants.LEDSegmentRange.CANdleBottom.index,
      LEDConstants.LEDSegmentRange.CANdleBottom.count,
      LEDConstants.Patterns.noPatternAnimation
    );
     LEDSegment CANdleFull = new LEDSegment(
      LEDConstants.LEDSegmentRange.CANdleFull.index,
      LEDConstants.LEDSegmentRange.CANdleFull.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    // LEDSegment Strip1 = new LEDSegment(      // strip not currently on robot
    //   LEDConstants.LEDSegmentRange.Strip1.index,
    //   LEDConstants.LEDSegmentRange.Strip1.count,
    //   LEDConstants.Patterns.noPatternAnimation
    // );
    // LEDSegment Full = new LEDSegment(
    //   LEDConstants.LEDSegmentRange.Full.index,
    //   LEDConstants.LEDSegmentRange.Full.count,
    //   LEDConstants.Patterns.noPatternAnimation
    // );
    
    segments.put(LEDSegmentRange.CANdleTop, CANdleTop);
    segments.put(LEDSegmentRange.CANdleBottom, CANdleBottom);
    // segments.put(LEDSegmentRange.Strip1, Strip1);  // strip not currently on robot
    // segments.put(LEDSegmentRange.Full, Full);
    segments.put(LEDSegmentRange.CANdleFull, CANdleFull);
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
   * @param color the color to use
   * @param segment the segment to use
   */
  public void setColor(Color color, LEDSegmentRange segment) {
    Color[] pattern = {color};
    setPattern(pattern, segment);
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

  public void setPattern(Color[] pattern, Color edgeColor, int edgeWidth, LEDSegmentRange segment) {
    if (pattern.length < (edgeWidth*2)+1) return;

    setLEDs(edgeColor, segment.index, edgeWidth);
    setLEDs(edgeColor, segment.count + segment.index-edgeWidth, edgeWidth);
    
    for (int indexLED = edgeWidth, indexPattern = 0; indexLED < segment.count-edgeWidth; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
  }

  /**
   * Sets the animation for a given certain led segment
   * @param animation animation to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    if (segments.containsKey(segment)) {
      segments.get(segment).setAnimation(animation, loop);
    } else if (segment == LEDSegmentRange.Full) {
      // Special case
      segments.get(LEDSegmentRange.CANdleTop).setAnimation(animation, loop);
      segments.get(LEDSegmentRange.CANdleBottom).setAnimation(animation, loop);
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
   * Sets LEDs using BCR color enum (ex: IDLE)
   * @param color color to set
   */
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.b);
  }
  
  /**
   * Sets LEDs using robot state (ex: IDLE)
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
  }

  /**
   * 
   * @param segment
   */
  public void updateStateLEDs(LEDSegmentRange segment) {
    // Store the state for periodics
    currentState = robotState.getState();
    // Set LEDs to match the state, as defined in Constants.BCRColor
    switch (currentState) {
    case IDLE:
      if (feeder.isPiecePresent()) {
        setLEDs(255, 30, 0, segment.index, segment.count);
      }
      else {
        setLEDs(BCRColor.IDLE, segment.index, segment.count);
      }
      break;
    case INTAKING:
      setLEDs(BCRColor.INTAKING, segment.index, segment.count);
      break;
    case SHOOTING:
      setLEDs(BCRColor.SHOOTING, segment.index, segment.count);
      break;
    }
  }


  @Override
  public void periodic() {

    // if(degreesFromSpeaker <= accuracyDisplayThreshold){
    //   accuracy = (int)((accuracyDisplayPattern.length/2)*(1-((degreesFromSpeaker-1)/accuracyDisplayThreshold)));
    //   if (accuracy > (accuracyDisplayPattern.length/2)) {
    //     setColor(Color.kGreen, LEDSegmentRange.Strip1);
    //   } else {
    //     setPattern(accuracyDisplayPattern, Color.kGreen, accuracy, LEDSegmentRange.Strip1);
    //   }
    // }

    // Every scheduler run, update the animations for all segments
    if(RobotPreferences.isStickyFaultActive()) segments.get(LEDSegmentRange.CANdleBottom).setEdgeColor(Color.kRed);
    else segments.get(LEDSegmentRange.CANdleBottom).setEdgeColor(Color.kBlack);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      // Display this segments
      LEDSegment segment = segments.get(segmentKey);
      if(segment.getEdgeColor() != Color.kBlack && segment.getEdgeColor() != null){
        setPattern(segments.get(segmentKey).getCurrentFrame(), segment.getEdgeColor(), 1, segmentKey);
      } else {
        setPattern(segments.get(segmentKey).getCurrentFrame(), segmentKey);
      }
      // Move to the next frame
      shouldClear = segments.get(segmentKey).advanceFrame();
      if (shouldClear) {
        updateStateLEDs(segmentKey);
      }
    }
    updateStateLEDs(LEDSegmentRange.CANdleFull);
  }
}