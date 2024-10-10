// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;

import static frc.robot.Constants.Ports.CANDriveBackLeftMotor;
import static frc.robot.Constants.Ports.CANDriveBackRightMotor;

import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

public class DrivePercentSpeed extends Command {
  /** Creates a new DriveAtVoltage. */
  private DriveTrain driveTrain;
  private FileLog log;
  private double angleFacing, percentSpeed, time;
  private final Timer timer = new Timer();
  /**
   * @param time max time in seconds that the robot will drive for, from smartdashboard
   * @param voltage desired voltage to run each motor at, from smartdashboard
   */

  public DrivePercentSpeed(double angleFacing, double percentSpeed, double time, DriveTrain driveTrain, FileLog log) {
    // Use addRequirements() here to declare subsystem dependencies.

    this.driveTrain = driveTrain;
    this.log = log;
    this.angleFacing = angleFacing;
    this.percentSpeed= percentSpeed;
    this.time = time;
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    driveTrain.setDriveModeCoast(false);
    log.writeLog(false, "DrivePercentSpeed", "Initialize", "Angle Facing Desired", angleFacing, "Percent Speed Deisred", percentSpeed);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //log.writeLog(false, "DrivePercentSpeed", "execute");
   
    driveTrain.setWheelFacings(angleFacing);
    driveTrain.setDriveMotorsOutput(percentSpeed);
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveTrain.stopMotors();
    log.writeLog(false, "DrivePercentSpeed", "End");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }
}
