// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.Sequences.*;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpThreePieceShoot extends SequentialCommandGroup {
  /** Creates a new AmpTwoPieceShoot. */
  public AmpThreePieceShoot(Intake intake, Shooter shooter, DriveTrain driveTrain, Feeder feeder, Wrist wrist, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
            new SequentialCommandGroup(          
                  new DriveResetPose(0.7, 1.3, -60, false, driveTrain, log),
                  new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToAmpCloseNoteRed.value], driveTrain, log)
            ),
            new SequentialCommandGroup(
                  new DriveResetPose(0.7, 6.5, 60, false, driveTrain, log),
                  new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToAmpCloseNoteBlue.value], driveTrain, log)
            ),
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ShooterSetPercent(-0.02, shooter, log),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpNoteToFarNoteRed.value], driveTrain, log), 
            new DriveToPose(new Pose2d(3.0, 1.2, new Rotation2d(-26)), driveTrain, log)
          ),
          new SequentialCommandGroup(
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpNoteToFarNoteBlue.value], driveTrain, log),
            new DriveToPose(new Pose2d(3.0, 7, new Rotation2d(26)), driveTrain, log)
          ),
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ShooterSetPercent(-0.02, shooter, log),
      new WristSetAngle(WristAngle.lowerLimit, wrist, log)
    );
  }
}
