// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.FileLog;

public class IntakeStop extends Command {

  private final Intake intake;
  private final FileLog log;

  /**
   * Stops the intake and centering motors
   * @param intake intake subsystem
   * @param log
   */
  public IntakeStop(Intake intake, FileLog log) {
    this.intake = intake;
    this.log = log;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.stopIntakeMotor();
    intake.stopCenteringMotor();

    log.writeLog(false, "IntakeStop", "Initialize");
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
