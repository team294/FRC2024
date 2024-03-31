// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.CANifier.LEDChannel;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants;
import frc.robot.Constants.ShooterConstants;
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
  private DriveTrain driveTrain;
  private Shooter shooter;
  private Feeder feeder;
  private boolean shouldClear;
  private double degreesFromSpeaker;
  private int halfAccuracyLEDs;
  private Timer timer;

  // private Color[] accuracyDisplayPattern = {Color.kRed, Color.kRed};
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  /**
   * Creates the CANdle LED subsystem.
   * @param CANPort
   * @param subsystemName
   * @param shooter
   * @param feeder
   * @param robotState
   * @param log
   * @param timer
   */
  public LED(int CANPort, String subsystemName, DriveTrain driveTrain, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log, Timer timer) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.driveTrain = driveTrain;
    this.shooter = shooter;
    this.feeder = feeder;
    this.shouldClear = false;
    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;
    this.timer = timer;

    // Create the LED segments
    for (LEDSegmentRange segment : LEDSegmentRange.values()) {
      segments.put(segment, new LEDSegment(segment.index, segment.count, LEDConstants.Patterns.noPatternAnimation));
    }
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
    log.writeLog(false, "LED", "Clear LEDs");
  }

  /**
   * Clear the LEDs of a specific segment range
   * @param segment the segment to clear
   */
  public void clearLEDs(LEDSegmentRange segment) {
    setLEDs(0, 0, 0, segment);
    log.writeLog(false, "LED", "Clear LEDs");
  }
  
  /**
   * Clear all animation
   */
  public void clearAnimation() {
    candle.clearAnimation(0);
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      setAnimation(LEDConstants.Patterns.noPatternAnimation, segmentKey, false);
    }
    log.writeLog(false, "LED", "Clear Animation");
  }
  
  /**
   * Start a built-in animation
   * @param anim
   */
  public void animate(Animation anim) {
    candle.animate(anim);
    log.writeLog(false, "LED", "Animate");
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param color the color to use
   * @param segment the segment to use
   */
  public void setColor(Color color, LEDSegmentRange segment) {
    Color[] pattern = {color};
    setPattern(pattern, segment);
    log.writeLog(false, "LED", "Set Color");
  }

  /**
   * Sets the pattern and resizes it to fit the LED strip
   * @param pattern the pattern to use
   * @param segment the segment to use
   */
  public void setPattern(Color[] pattern, LEDSegmentRange segment) {
    if (pattern.length == 0) return;
    for (int indexLED = 0, indexPattern = 0; indexLED < segment.count; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
    log.writeLog(false, "LED", "Set Pattern");
  }

  /**
   * Sets the animation for a given led segment
   * @param animation animation to display
   * @param segment segment to play animation on
   * @param loop whether the animation repeats
   */
  public void setAnimation(Color[][] animation, LEDSegmentRange segment, boolean loop) {
    segments.get(segment).setAnimation(animation, loop);
    log.writeLog(false, "LED", "Set Animation");
  }

  public void setAnimation(Color[] pattern, LEDSegmentRange segment, boolean loop) {
    Color[][] anim = {pattern};
    segments.get(segment).setAnimation(anim, loop);
    log.writeLog(false, "LED", "Set Animation");
  }
  
  public void setAnimation(Color color, LEDSegmentRange segment) {
    segments.get(segment).setAnimation(color);
    log.writeLog(false, "LED", "Set Animation");
  }
  
  public void setAnimation(BCRColor color, LEDSegmentRange segment) {
    Color _color = new Color(color.r, color.g, color.b);
    segments.get(segment).setAnimation(_color);
    log.writeLog(false, "LED", "Set Animation");
  }

  /**
   * Sets LEDs using only R, G, and B
   * @param r red value
   * @param g green value
   * @param b blue value
   */
  public void setLEDs(int r, int g, int b) {
    candle.setLEDs(r, g, b);
    log.writeLog(false, "LED", "Set LEDs");
  }

  /**
   * Takes in color and sets correct RGB values
   * @param color color to set
   */
  public void setLEDs(Color color) {
    setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255));
    log.writeLog(false, "LED", "Set LEDs");
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
    log.writeLog(false, "LED", "Set LEDs");
  }
  /**
   * Sets LEDs using color and index values
   * @param color color to set
   * @param index index to start at
   */
  public void setLEDs(Color color, int index) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, 1);
    log.writeLog(false, "LED", "Set LEDs");
  }
  /**
   * Sets LEDs using color, index, and count values
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(Color color, int index, int count) {
    candle.setLEDs((int) (color.red * 255), (int) (color.green * 255), (int) (color.blue * 255), 0, index, count);
    log.writeLog(false, "LED", "Set LEDs");
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
    log.writeLog(false, "LED", "Set LEDs");
  }
  /**
   * Sets LEDs using BCR color enum (ex: IDLE)
   * @param color color to set
   */
  public void setLEDs(BCRColor color) {
    candle.setLEDs(color.r, color.g, color.b);
    log.writeLog(false, "LED", "Set LEDs");
  }
  
  /**
   * Sets LEDs using robot state (ex: IDLE)
   * @param color color to set
   * @param index index to start at
   * @param count number of LEDs
   */
  public void setLEDs(BCRColor color, int index, int count) {
    candle.setLEDs(color.r, color.g, color.b, 0, index, count);
    log.writeLog(false, "LED", "Set LEDs");
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
        if(shooter.isVelocityControlOn() && Math.abs(shooter.getTopShooterVelocityPIDError()) < ShooterConstants.velocityErrorTolerance
        && (segment == LEDSegmentRange.StripLeft || segment == LEDSegmentRange.StripRight)) {
          setAnimation(Color.kGreen, segment);
        } else if (shooter.getTopShooterTargetRPM() > 0 && (segment == LEDSegmentRange.StripLeft || segment == LEDSegmentRange.StripRight))  {
          Double percent = shooter.getTopShooterVelocity() / shooter.getTopShooterTargetRPM();
          Color[] segmentPattern = new Color[segment.count];
          if (segment == LEDSegmentRange.StripLeft) {
            for (int i = 0; i < segment.count; i++) {
              if (i >= (1.0 - percent) * segment.count) {
                segmentPattern[i] = Color.kPurple;
              } else {
                segmentPattern[i] = Color.kOrange;
              }
            }
          } else if (segment == LEDSegmentRange.StripRight) {
            for (int i = 0; i < segment.count; i++) {
              if (i <= percent * segment.count) {
                segmentPattern[i] = Color.kPurple;
              } else {
                segmentPattern[i] = Color.kOrange;
              }
            }
          }
          setAnimation(segmentPattern, segment, true);
        } else {
          setAnimation(Color.kOrange, segment);
        }
      }
      else {
        setAnimation(BCRColor.IDLE, segment);
      }
      break;
    case INTAKING:
      setAnimation(BCRColor.INTAKING, segment);
      break;
    case SHOOTING:
      setAnimation(BCRColor.SHOOTING, segment);
      break;
    }
    log.writeLog(false, "LED", "Update State LEDs", "State", currentState);
  }

  private void DisplayLEDs() {
    for (LEDSegmentRange segmentKey : segments.keySet()) {
      // Display this segments
      LEDSegment segment = segments.get(segmentKey);
      setPattern(segment.getCurrentFrame(), segmentKey);
      
      // Move to the next frame
      shouldClear = segment.advanceFrame();
      if (shouldClear) {
        updateStateLEDs(segmentKey);
      }
    }
  }

  @Override
  public void periodic() {
    updateStateLEDs(LEDSegmentRange.Full);

    if(RobotPreferences.isStickyFaultActive()) {
      segments.get(LEDSegmentRange.CANdleFull).setAnimation(Color.kRed);
    }

    // if (degreesFromSpeaker <= LEDConstants.accuracyDisplayThreshold){
    //   numAccuracyLEDs = (int)(((int)(LEDSegmentRange.Strip1.count/2))*(1-((degreesFromSpeaker-1)/LEDConstants.accuracyDisplayThreshold)));
    //   if (numAccuracyLEDs > (LEDSegmentRange.Strip1.count/2)) {
    //     setColor(Color.kGreen, LEDSegmentRange.Strip1);
    //   } else {
    //     setPattern(LEDConstants.Patterns.accuracyDisplayPattern, Color.kGreen, numAccuracyLEDs, LEDSegmentRange.Strip1);
    //   }
    // }
    
    degreesFromSpeaker = driveTrain.getAngleErrorToSpeaker();

    if (degreesFromSpeaker <= LEDConstants.accuracyDisplayThreshold){
      LEDSegmentRange horizontalSegment = LEDSegmentRange.StripHorizontal;
      halfAccuracyLEDs = ((int)horizontalSegment.count/2)*((int)(1-((degreesFromSpeaker)/LEDConstants.accuracyDisplayThreshold)));
      Color[] accuracyArray = new Color[horizontalSegment.count];
      for(int index = 0; index < horizontalSegment.count; index++){
        if(index < halfAccuracyLEDs || index >= (horizontalSegment.count-halfAccuracyLEDs)){accuracyArray[index] = Color.kGreen;}
        else{accuracyArray[index] = Color.kRed;}
      }
      segments.get(horizontalSegment).setAnimation(accuracyArray);
    }

    DisplayLEDs();
  }
}