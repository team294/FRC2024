// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.LEDConstants.LEDSegment;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class CANdleTeamFlash extends Command {
  private LED led;
  private FileLog log;
  private boolean fromShuffleboard;
  private Color color = new Color();
  private double t;
  private double executionsPerChange;
  private LEDSegment segment;
  /** Creates a new CANdleStop. */


  public CANdleTeamFlash(LED led, LEDSegment segment, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.segment = segment;
    this.log = log;
    fromShuffleboard = true;

    addRequirements(led);
  }

  public CANdleTeamFlash(LED led, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.segment = LEDSegment.Full;
    this.log = log;
    fromShuffleboard = true;

    addRequirements(led);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(executionsPerChange > 1){
        if (t > 8) {
            led.setLEDs(255,30,0);
        }
        else {
            led.setLEDs(0,255,255);
        }
        // color = new Color(r, g, b);
        // led.setLEDs(color, 0);
        t += 1;
        if (t > 16) {
            t = 0;
        }
      executionsPerChange = 0;
    } else {
      executionsPerChange++;
    }
  }

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

  // Allows for running while robot is disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}