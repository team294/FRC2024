// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.subsystems.DriveTrain;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import frc.robot.utilities.AllianceSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ResetPoseAndDriveToPosAuto extends ConditionalCommand {
  /**
   * Reset pose and drives based on a given trajectory depending on alliance
   * @param redPos A Pose2d used to reset the pose when on the red alliance
   * @param bluePose A Pose2d used to reset the pose when on the blue alliance
   * @param trajectoryRed trajectory to follow if on the Red Alliance
   * @param trajectoryBlue trajectory to follow if on the Blue Alliance
   * @param drivetrain
   * @param feeder
   * @param shooter
   * @param wrist
   * @param intake
   * @param robotState
   * @param cache
   * @param alliance
   * @param log
   */
  public ResetPoseAndDriveToPosAuto(Pose2d redPos, Pose2d bluePos, TrajectoryType trajectory, DriveTrain drivetrain, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add the deadline command in the super() call. 
    // Add other commands using addCommands().
    super(
      new SequentialCommandGroup(
        new DriveResetPose(redPos, false, drivetrain, log),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[trajectory.value][TrajectoryConstants.RED], drivetrain, log)
      ),
      new SequentialCommandGroup(
        new DriveResetPose(bluePos, false, drivetrain, log),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[trajectory.value][TrajectoryConstants.BLUE], drivetrain, log)
      ),
      () -> alliance.getAlliance() == Alliance.Red
    );
  }
}
