// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;

public class SetOdometryUsingVision extends Command {

  private final DriveTrain driveTrain;

  private final boolean odometryUsingVision;

  /** Creates a new SetOdometryUsingVision. */
  public SetOdometryUsingVision(boolean enabled, DriveTrain driveTrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrain = driveTrain;
    odometryUsingVision = enabled;

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setVisionForOdomoetryState(odometryUsingVision);
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

  // Allows for running while robot is disabled
  @Override
  public boolean runsWhenDisabled() {
    return true;
  }
}
