// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

public class ToggleCoast extends Command {
    
    private final DriveTrain driveTrain;
    private final FileLog log;

/**
    Untested. Adds a button on Shuffleboard which allows driver to toggle between coast and break drive modes
*/
public ToggleCoast(DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;

    addRequirements(driveTrain);
}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(SmartDashboard.getBoolean("Toggle Coast Drive", false));
    log.writeLog(true, "Toggle Coast", "Drive Mode Toggled from Shuffleboard");
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
