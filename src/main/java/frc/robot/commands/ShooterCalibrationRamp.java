// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;

public class ShooterCalibrationRamp extends Command {
  private final Shooter shooter;
  private final FileLog log;

  private double volts;
  /** Creates a new FeedForwardTest. */
  public ShooterCalibrationRamp(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    volts = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.writeLog(false, shooter.getName(), "Increasing motor voltage",
      "Volts", volts,
      "Velocity", shooter.getShooterVelocity()
    );
    shooter.setVoltage(volts);
    // Every 20ms, increase volts by 0.01
    volts += 0.005;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do NOT exceed 5 volts for safety
    return volts > 4.0;
  }
}
