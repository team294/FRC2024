/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.RobotDimensions;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.FileLog;

public class WristOverHeadSetAngleWithVision extends Command {

  private double angle;
  private final AllianceSelection allianceSelection;
  private final DriveTrain driveTrain;
  private final Wrist wrist;
  private final FileLog log;

  /**
   * Moves wrist to target angle.  Command ends when wrist is within 5 degrees of the target position.
   * <p> This command does nothing and immediately returns if the wrist is not calibrated.
   * @param angle target angle in degrees.  (0 = horizontal in front of robot, + = up, - = down)
   */
  public WristOverHeadSetAngleWithVision(Wrist wrist, AllianceSelection allianceSelection, DriveTrain drivetrain, FileLog log) {
    this.wrist = wrist;
    this.allianceSelection = allianceSelection;
    this.driveTrain = drivetrain;
    this.log = log;

    SmartDashboard.putNumber("Wrist Vision Constant Offset", 0);

    addRequirements(wrist);
  }

  /*
   * @param int n the number of recursions the function runs before using base condition
   */
  private double getAngleFromDistance(int n) {
    if (n == 0) return 0;

    double x = driveTrain.getPose().getX();
    double y = (driveTrain.getPose().getY() - allianceSelection.getSpeakerYPos());
    // distance from speaker (robot distance + arm distance from center of robot)
    double dist = Math.sqrt(x*x+y*y) + RobotDimensions.lengthOfArmFromWristPivotToCenterPathOfShooter*Math.cos(Units.degreesToRadians(getAngleFromDistance(n-1)));
    // height of shooter: height of robot + height of arm
    double heightOfShooter = RobotDimensions.heightFromGroundToWristPivot+RobotDimensions.lengthOfArmFromWristPivotToCenterPathOfShooter*Math.sin(Units.degreesToRadians(getAngleFromDistance(n-1)));

    return 90 - (Units.radiansToDegrees(Math.atan((FieldConstants.heightOfSpeaker-heightOfShooter)/dist))) - 10;
  } 

  // Called just before this Command runs the first time
  @Override
  public void initialize() {
    log.writeLog(false, "WristSetAngleWithVision", "Initialize", "Target", angle);
  }

  // Called repeatedly when this Command is scheduled to run
  @Override
  public void execute() {
    angle = getAngleFromDistance(3);
    wrist.setWristAngle(angle - 3 + SmartDashboard.getNumber("Wrist Vision Constant Offset", 0));
    wrist.updateWristLog(false);
  }

  // Called once after isFinished returns true
  @Override
  public void end(boolean interrupted) {
    log.writeLog(false, "WristSetAngleWithVision", "End", "Target", angle, "Current angle", wrist.getWristAngle());
  }

  // Make this return true when this Command no longer needs to run execute()
  @Override
  public boolean isFinished() {
    return !wrist.isEncoderCalibrated();
  }
}