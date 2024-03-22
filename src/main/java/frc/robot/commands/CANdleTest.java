// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.LED;
import frc.robot.utilities.FileLog;

public class CANdleTest extends Command {

  private LED led;
  private FileLog log;
  private boolean fromShuffleboard;
  private int r, g, b, w, index, count;

  /** Creates a new CANdleTest. */
  public CANdleTest(LED led, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.log = log;
    fromShuffleboard = true;
    // Use addRequirements() here to declare subsystem dependencies.
    SmartDashboard.putNumber("CANdleR", 0);
    SmartDashboard.putNumber("CANdleG", 0);
    SmartDashboard.putNumber("CANdleB", 0);
    SmartDashboard.putNumber("CANdleIndex", 0);
    SmartDashboard.putNumber("CANdleCount", 1);

    addRequirements(led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.r = (int) SmartDashboard.getNumber("CANdleR", 0);
    this.g = (int) SmartDashboard.getNumber("CANdleG", 0);
    this.b = (int) SmartDashboard.getNumber("CANdleB", 0);
    this.index = (int) SmartDashboard.getNumber("CANdleIndex", 0);
    this.count = (int) SmartDashboard.getNumber("CANdleCount", 0);
    led.setLEDs(r, g, b, index, count);
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
