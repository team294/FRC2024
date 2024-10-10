// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

import static frc.robot.Constants.Ports.CANDriveBackLeftMotor;
import static frc.robot.Constants.Ports.CANDriveBackRightMotor;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class DriveAtVoltage extends Command {
  /** Creates a new DriveAtVoltage. */
  private DriveTrain driveTrain;
  private FileLog log;
  private double time, voltage;
  /**
   * @param time max time in seconds that the robot will drive for, from smartdashboard
   * @param voltage desired voltage to run each motor at, from smartdashboard
   */

  public DriveAtVoltage(DriveTrain driveTrain, FileLog log, double voltage, double time) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
    this.driveTrain = driveTrain;
    this.log = log;
    this.time = time;
    this.voltage = voltage;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    final VoltageOut directSwerveMotorControl = new VoltageOut(0.0);
    final PositionVoltage directSwerveTurnMotorControl = new PositionVoltage(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    log.writeLog(false, "DriveAtVoltage", "");
    driveTrain.setModuleStates(null, isScheduled());
    
    CANDriveBackLeftMotor.setControl(directSwerveMotorControl.withOutput(voltage));
    CANDriveBackRightMotor.setControl(directSwerveMotorControl.withOutput(voltage));
    CANDriveFrontLeftMotor.setControl(directSwerveMotorControl.withOutput(voltage));
    CANDriveFrontRightMotor.setControl(directSwerveMotorControl.withOutput(voltage));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
