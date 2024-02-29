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

  private double percent;

  /** Creates a new FeedForwardTest. */
  public ShooterCalibrationRamp(Shooter shooter, FileLog log) {
    this.shooter = shooter;
    this.log = log;

    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    percent = 0.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.writeLog(false, shooter.getName(), "Increasing motor voltage",
      "Top Volts", shooter.getShooterTopVoltage(),
      "Top Velocity", shooter.getShooter1Velocity(),
      "Bottom Volts", shooter.getShooterBottomVoltage(),
      "Bottom Velocity", shooter.getShooter2Velocity()
    );

    // Every 20ms, increase percent slightly
    percent += 0.0004;
    shooter.setMotorPercentOutput(percent, 0.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do NOT exceed 35%  for safety
    return percent > 0.35;
  }
}
