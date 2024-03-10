// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterSetVelocity extends Command {
  private double velocity = 0.0;
  private final FileLog log;
  private final Shooter shooter;
  private final VelocityType type;
  private boolean fromShuffleboard;
  private int counter;

  public enum VelocityType{
    immediatelyEnd,
    runForever, 
    waitForVelocity
  }

  /**
   * Sets the shooter wheel velocity (rpm) for top and bottom shooter motors.
   * @param velocity wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param type ShooterSetVelocity.VelocityType = immediatelyEnd, runForever, or waitForVelocity
   * @param shooter shooter subsystem
   * @param log
   */
  public ShooterSetVelocity(double velocity, VelocityType type, Shooter shooter, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocity = velocity;
    this.type = type;
    this.log = log;
    this.shooter = shooter;
    this.fromShuffleboard = false;
    addRequirements(shooter);
  }

  /**
   * Sets the shooter wheel velocity (rpm) based upon input from Shuffleboard (+ = shoot forward, - = backwards).
   * @param type ShooterSetVelocity.VelocityType = immediatelyEnd, runForever, or waitForVelocity
   * @param shooter shooter subsystem
   * @param log
   */
  public ShooterSetVelocity(VelocityType type, Shooter shooter, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.velocity = 0.0;
    this.type = type;
    this.log = log;
    this.shooter = shooter;
    this.fromShuffleboard = true;
    addRequirements(shooter);

    if(SmartDashboard.getNumber("Shooter velocity", -9999.9) == -9999.9) {
      SmartDashboard.putNumber("Shooter velocity", 0);
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (type == VelocityType.waitForVelocity) {
      shooter.enableFastLogging(true);
    }

    if (fromShuffleboard) {
      velocity = SmartDashboard.getNumber("Shooter velocity", 0.0);
    }
    shooter.setShooterVelocity(velocity);
    log.writeLog(false, "ShooterSetVelocity", "Initialize", 
      "Velocity", velocity, "Velocity Type", type.toString());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.enableFastLogging(false);
    log.writeLog(false, "ShooterSetVelocity", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    switch (type){
      case immediatelyEnd:
        return true;
      case runForever:
        return false;
      case waitForVelocity:
        if(Math.abs(shooter.getTopShooterVelocity() - velocity) < ShooterConstants.velocityErrorTolerance){
          counter++;
        }else{
          counter = 0;
        }
        return counter >= 5;
      default:
        return true;
    }
  }
}