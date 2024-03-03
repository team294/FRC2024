/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.VisionConstants.JeVoisConstants;
import frc.robot.Constants.VisionConstants.PhotonVisionConstants;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.JeVoisCamera;
import frc.robot.subsystems.NotePhotonCameraWrapper;
import frc.robot.utilities.*;

/**
 * Command to drive to a note using the JeVois camera and the drive train
 */
public class DriveToNote extends Command {
  private final DriveTrain driveTrain;
  private final PIDController fwdRateController;
  private final PIDController leftRateController;
  private final PIDController turnRateController;
  private final FileLog log;
  private int logRotationKey;
  
  private double fwdVelocity, leftVelocity, turnRate;
  // private double lastFwdPercent, lastTime, curTime;

  // private final double maxFwdRateChange = 2.0;
  // private final double maxRevRateChange = -1.4;

  /**
   * @param driveTrain drive train subsystem to use
   * @param log filelog to use
   */
  public DriveToNote(DriveTrain driveTrain, FileLog log) {
    this.driveTrain = driveTrain;
    this.log = log;
    fwdRateController = new PIDController(0.4, 0, 0);
    leftRateController = new PIDController(0.005, 0, 0);
    turnRateController = new PIDController(0.2, 0, 0);
    fwdRateController.setSetpoint(PhotonVisionConstants.pitchSetpoint); // setpoint is the bottom of the screen
    leftRateController.setSetpoint(PhotonVisionConstants.yawSetpoint); // setpoint is the middle line on the screen
    turnRateController.setSetpoint(PhotonVisionConstants.yawSetpoint); // setpoint is the middle line on the screen

    logRotationKey = log.allocateLogRotation();

    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveTrain.setDriveModeCoast(false);

    // lastFwdPercent = 0;
    // lastTime = System.currentTimeMillis() / 1000.0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // curTime = System.currentTimeMillis() / 1000.0;

    PhotonPipelineResult latestResult = driveTrain.getLatestResult();
    if (!latestResult.hasTargets()) {
      if(log.isMyLogRotation(logRotationKey)) {
        log.writeLog(false, "DriveToNote", "No targets captured");
      }
        return;
    }

    PhotonTrackedTarget bestTarget = latestResult.getBestTarget();
    

    fwdVelocity = fwdRateController.calculate(bestTarget.getPitch());
    leftVelocity = leftRateController.calculate(bestTarget.getYaw());
    turnRate = turnRateController.calculate(bestTarget.getYaw());

    fwdVelocity = MathUtil.clamp(fwdVelocity, -SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxSpeedMetersPerSecond);
    leftVelocity = MathUtil.clamp(leftVelocity, -SwerveConstants.kMaxSpeedMetersPerSecond, SwerveConstants.kMaxSpeedMetersPerSecond);
    turnRate = MathUtil.clamp(leftVelocity, -SwerveConstants.kMaxTurningRadiansPerSecond, SwerveConstants.kMaxTurningRadiansPerSecond);

    if(log.isMyLogRotation(logRotationKey)) {
      log.writeLog(false, "DriveToNote", "Joystick", "Fwd", fwdVelocity, "Left", leftVelocity, "Turn", turnRate);
    }
    
    // double fwdRateChange = (fwdPercent - lastFwdPercent) / (curTime - lastTime);
    // if (fwdRateChange > maxFwdRateChange) {
    //   fwdPercent = lastFwdPercent + (curTime - lastTime)*maxFwdRateChange;
    // } else if (fwdRateChange < maxRevRateChange) {
    //   fwdPercent = lastFwdPercent +(curTime - lastTime)*maxRevRateChange;

    // }

    SmartDashboard.putNumber("DriveToNote fwd", fwdVelocity);
    SmartDashboard.putNumber("DriveToNote left", leftVelocity);
    SmartDashboard.putNumber("DriveToNote turnRate", turnRate);
    SmartDashboard.putNumber("DriveToNote BestTargetPitch", bestTarget.getPitch());
    SmartDashboard.putNumber("DriveToNote BestTargetYaw", bestTarget.getYaw());
    
    driveTrain.drive(fwdVelocity, leftVelocity, turnRate, true, false);

    // lastFwdPercent = fwdPercent;
    // lastTime = curTime;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}