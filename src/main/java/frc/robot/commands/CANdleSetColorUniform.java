// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CANdleSetColorUniform extends InstantCommand {
  LED led;
  FileLog log;
  int r, g, b;
  boolean fromSmartDashboard = false;

  /** Set the colour of all LEDs to the given rgb value
   * @param r the red portion of the colour (0-255)
   * @param g the green portion of the colour (0-255)
   * @param b the blue portion of the colour (0-255)
   * @param led the LED subsystem; it will light up
   * @param log the File Log
  */
  public CANdleSetColorUniform(int r, int g, int b, LED led, FileLog log) {
    // Usa addRequirements() here to declare subsystem dependencies.
    this.r = r;
    this.g = g;
    this.b = b;
    this.led = led;
    this.log = log;
    fromSmartDashboard = false;
  }

  /** Set the colour of all LEDs to the rgb value desired from SmartDashboard
   * @param led the LED subsystem; it will light up
   * @param log the File Log
   */
  public CANdleSetColorUniform(LED led, FileLog log) {
    // Will get the values from SmartDashboard
    SmartDashboard.putNumber("CANdleUniformR", 0);
    SmartDashboard.putNumber("CANdleUniformG", 0);
    SmartDashboard.putNumber("CANdleUniformB", 0);
    this.led = led;
    this.log = log;
    fromSmartDashboard = true;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (fromSmartDashboard) {
      // Update the values
      this.r = (int) SmartDashboard.getNumber("CANdleUniformR", 0);
      this.g = (int) SmartDashboard.getNumber("CANdleUniformG", 0);
      this.b = (int) SmartDashboard.getNumber("CANdleUniformB", 0);
    }
    led.setLEDs(r, g, b);
  }
}
