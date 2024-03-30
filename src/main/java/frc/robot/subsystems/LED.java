// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.HashMap;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
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
  private Shooter shooter;
  private Feeder feeder;
  private boolean shouldClear;
  private Timer timer;
  // private double accuracyDisplayThreshold;
  // private int accuracy;

  // private Color[] accuracyDisplayPattern = {Color.kRed, Color.kRed};

  // LED Segments
  //private LEDSegment CANdle;
  //private LEDSegment strip1;
  private HashMap<LEDSegmentRange, LEDSegment> segments;

  public LED(int CANPort, String subsystemName, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log, Timer timer) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.shooter = shooter;
    this.feeder = feeder;
    this.shouldClear = false;
    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;
    this.timer = timer;

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
    LEDSegment Strip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip1.index,
      LEDConstants.LEDSegmentRange.Strip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Strip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip2.index,
      LEDConstants.LEDSegmentRange.Strip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Strip1and2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip1and2.index,
      LEDConstants.LEDSegmentRange.Strip1and2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Full = new LEDSegment(
      LEDConstants.LEDSegmentRange.Full.index,
      LEDConstants.LEDSegmentRange.Full.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FirstTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FirstTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FirstTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FirstTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FirstTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FirstTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SecondTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SecondTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SecondTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SecondTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SecondTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SecondTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment ThirdTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.ThirdTenthStrip1.index,
      LEDConstants.LEDSegmentRange.ThirdTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment ThirdTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.ThirdTenthStrip2.index,
      LEDConstants.LEDSegmentRange.ThirdTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FourthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FourthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FourthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FourthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FourthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FourthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FifthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FifthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FifthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FifthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FifthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FifthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SixthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SixthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SixthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SixthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SixthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SixthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SeventhTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SeventhTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SeventhTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SeventhTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SeventhTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SeventhTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment EighthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.EighthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.EighthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment EighthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.EighthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.EighthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment NinthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.NinthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.NinthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment NinthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.NinthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.NinthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment TenthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.TenthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.TenthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment TenthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.TenthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.TenthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    
    
    segments.put(LEDSegmentRange.CANdleTop, CANdleTop);
    segments.put(LEDSegmentRange.CANdleBottom, CANdleBottom);
    segments.put(LEDSegmentRange.CANdleFull, CANdleFull);
    segments.put(LEDSegmentRange.Strip1, Strip1);
    segments.put(LEDSegmentRange.Strip2, Strip2);
    segments.put(LEDSegmentRange.Strip1and2, Strip1and2);
    segments.put(LEDSegmentRange.Full, Full);
    segments.put(LEDSegmentRange.FirstTenthStrip1, FirstTenthStrip1);
    segments.put(LEDSegmentRange.FirstTenthStrip2, FirstTenthStrip2);
    segments.put(LEDSegmentRange.SecondTenthStrip1, SecondTenthStrip1);
    segments.put(LEDSegmentRange.SecondTenthStrip2, SecondTenthStrip2);
    segments.put(LEDSegmentRange.ThirdTenthStrip1, ThirdTenthStrip1);
    segments.put(LEDSegmentRange.ThirdTenthStrip2, ThirdTenthStrip2);
    segments.put(LEDSegmentRange.FourthTenthStrip1, FourthTenthStrip1);
    segments.put(LEDSegmentRange.FourthTenthStrip2, FourthTenthStrip2);
    segments.put(LEDSegmentRange.FifthTenthStrip1, FifthTenthStrip1);
    segments.put(LEDSegmentRange.FifthTenthStrip2, FifthTenthStrip2);
    segments.put(LEDSegmentRange.SixthTenthStrip1, SixthTenthStrip1);
    segments.put(LEDSegmentRange.SixthTenthStrip2, SixthTenthStrip2);
    segments.put(LEDSegmentRange.SeventhTenthStrip1, SeventhTenthStrip1);
    segments.put(LEDSegmentRange.SeventhTenthStrip2, SeventhTenthStrip2);
    segments.put(LEDSegmentRange.EighthTenthStrip1, EighthTenthStrip1);
    segments.put(LEDSegmentRange.EighthTenthStrip2, EighthTenthStrip2);
    segments.put(LEDSegmentRange.NinthTenthStrip1, NinthTenthStrip1);
    segments.put(LEDSegmentRange.NinthTenthStrip2, NinthTenthStrip2);
    segments.put(LEDSegmentRange.TenthTenthStrip1, TenthTenthStrip1);
    segments.put(LEDSegmentRange.TenthTenthStrip2, TenthTenthStrip2);
  }

  /**
   * Creates the CANdle LED subsystem.
   * @param subsystemName
   * @param log
   */
  public LED(int CANPort, String subsystemName, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log) {
    this.log = log;
    this.subsystemName = subsystemName;
    this.candle = new CANdle(CANPort, "");
    this.segments = new HashMap<LEDSegmentRange, LEDSegment>();
    this.robotState = robotState;
    this.currentState = BCRRobotState.State.IDLE;
    this.shooter = shooter;
    this.feeder = feeder;
    this.shouldClear = false;
    // this.accuracyDisplayThreshold = 35;
    // this.accuracy = 0;

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
    LEDSegment Strip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip1.index,
      LEDConstants.LEDSegmentRange.Strip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Strip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip2.index,
      LEDConstants.LEDSegmentRange.Strip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Strip1and2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.Strip1and2.index,
      LEDConstants.LEDSegmentRange.Strip1and2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment Full = new LEDSegment(
      LEDConstants.LEDSegmentRange.Full.index,
      LEDConstants.LEDSegmentRange.Full.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FirstTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FirstTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FirstTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FirstTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FirstTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FirstTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SecondTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SecondTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SecondTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SecondTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SecondTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SecondTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment ThirdTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.ThirdTenthStrip1.index,
      LEDConstants.LEDSegmentRange.ThirdTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment ThirdTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.ThirdTenthStrip2.index,
      LEDConstants.LEDSegmentRange.ThirdTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FourthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FourthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FourthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FourthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FourthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FourthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FifthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FifthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.FifthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment FifthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.FifthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.FifthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SixthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SixthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SixthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SixthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SixthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SixthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SeventhTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SeventhTenthStrip1.index,
      LEDConstants.LEDSegmentRange.SeventhTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment SeventhTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.SeventhTenthStrip2.index,
      LEDConstants.LEDSegmentRange.SeventhTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment EighthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.EighthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.EighthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment EighthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.EighthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.EighthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment NinthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.NinthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.NinthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment NinthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.NinthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.NinthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment TenthTenthStrip1 = new LEDSegment(
      LEDConstants.LEDSegmentRange.TenthTenthStrip1.index,
      LEDConstants.LEDSegmentRange.TenthTenthStrip1.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    LEDSegment TenthTenthStrip2 = new LEDSegment(
      LEDConstants.LEDSegmentRange.TenthTenthStrip2.index,
      LEDConstants.LEDSegmentRange.TenthTenthStrip2.count,
      LEDConstants.Patterns.noPatternAnimation
    );
    
    
    segments.put(LEDSegmentRange.CANdleTop, CANdleTop);
    segments.put(LEDSegmentRange.CANdleBottom, CANdleBottom);
    segments.put(LEDSegmentRange.CANdleFull, CANdleFull);
    segments.put(LEDSegmentRange.Strip1, Strip1);
    segments.put(LEDSegmentRange.Strip2, Strip2);
    segments.put(LEDSegmentRange.Strip1and2, Strip1and2);
    segments.put(LEDSegmentRange.Full, Full);
    segments.put(LEDSegmentRange.FirstTenthStrip1, FirstTenthStrip1);
    segments.put(LEDSegmentRange.FirstTenthStrip2, FirstTenthStrip2);
    segments.put(LEDSegmentRange.SecondTenthStrip1, SecondTenthStrip1);
    segments.put(LEDSegmentRange.SecondTenthStrip2, SecondTenthStrip2);
    segments.put(LEDSegmentRange.ThirdTenthStrip1, ThirdTenthStrip1);
    segments.put(LEDSegmentRange.ThirdTenthStrip2, ThirdTenthStrip2);
    segments.put(LEDSegmentRange.FourthTenthStrip1, FourthTenthStrip1);
    segments.put(LEDSegmentRange.FourthTenthStrip2, FourthTenthStrip2);
    segments.put(LEDSegmentRange.FifthTenthStrip1, FifthTenthStrip1);
    segments.put(LEDSegmentRange.FifthTenthStrip2, FifthTenthStrip2);
    segments.put(LEDSegmentRange.SixthTenthStrip1, SixthTenthStrip1);
    segments.put(LEDSegmentRange.SixthTenthStrip2, SixthTenthStrip2);
    segments.put(LEDSegmentRange.SeventhTenthStrip1, SeventhTenthStrip1);
    segments.put(LEDSegmentRange.SeventhTenthStrip2, SeventhTenthStrip2);
    segments.put(LEDSegmentRange.EighthTenthStrip1, EighthTenthStrip1);
    segments.put(LEDSegmentRange.EighthTenthStrip2, EighthTenthStrip2);
    segments.put(LEDSegmentRange.NinthTenthStrip1, NinthTenthStrip1);
    segments.put(LEDSegmentRange.NinthTenthStrip2, NinthTenthStrip2);
    segments.put(LEDSegmentRange.TenthTenthStrip1, TenthTenthStrip1);
    segments.put(LEDSegmentRange.TenthTenthStrip2, TenthTenthStrip2);
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
   * Set the pattern and resizes it to fit the LED strip
   * @param color the color to use
   * @param segment the segment to use
   */
  public void setColor(Color color, LEDSegmentRange segment) {
    Color[] pattern = {color};
    setPattern(pattern, segment);
    log.writeLog(false, "LED", "Set Color");
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
    log.writeLog(false, "LED", "Set Pattern");
  }

  public void setPattern(Color[] pattern, Color edgeColor, int edgeWidth, LEDSegmentRange segment) {
    if (pattern.length < (edgeWidth*2)+1) return;

    setLEDs(edgeColor, segment.index, edgeWidth);
    setLEDs(edgeColor, segment.count + segment.index-edgeWidth, edgeWidth);
    
    for (int indexLED = edgeWidth, indexPattern = 0; indexLED < segment.count-edgeWidth; indexLED++, indexPattern++) {
      if (indexPattern >= pattern.length) indexPattern = 0;
      setLEDs(pattern[indexPattern], segment.index + indexLED);
    }
    log.writeLog(false, "LED", "Set Pattern");
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
    } else if (segment == LEDSegmentRange.Strip1) {
      // Special case
      segments.get(LEDSegmentRange.CANdleTop).setAnimation(animation, loop);
      segments.get(LEDSegmentRange.CANdleBottom).setAnimation(animation, loop);
    }
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
        if(shooter.isVelocityControlOn() && Math.abs(shooter.getTopShooterVelocityPIDError()) < ShooterConstants.velocityErrorTolerance){
          setLEDs(150, 0, 255);
        } else {
          setLEDs(255, 30, 0, segment.index, segment.count);
        }
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
    log.writeLog(false, "LED", "Update State LEDs", "State", currentState);
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
    if(RobotPreferences.isStickyFaultActive()) segments.get(LEDSegmentRange.CANdleFull).setEdgeColor(Color.kRed);
    else segments.get(LEDSegmentRange.CANdleFull).setEdgeColor(Color.kBlack);
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
    updateStateLEDs(LEDSegmentRange.Full);
    if (timer.hasElapsed(125)) { // 2m 5s from start of teleop: 10 seconds left
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip2);
      new WaitCommand(1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.TenthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.TenthTenthStrip2);
      new WaitCommand(.5);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FirstTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SecondTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.ThirdTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FourthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.FifthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SixthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.SeventhTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.EighthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.NinthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.TenthTenthStrip1);
      setLEDs(255, 0, 0, LEDSegmentRange.TenthTenthStrip2);
      setLEDs(255, 0, 0, LEDSegmentRange.CANdleFull);
    }
  }
}