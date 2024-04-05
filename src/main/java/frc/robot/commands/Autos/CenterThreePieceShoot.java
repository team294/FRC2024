// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.Sequences.*;
import frc.robot.subsystems.*;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterThreePieceShoot extends SequentialCommandGroup {
  /** Creates a new SourceCenterThreePieceShoot. */
  public CenterThreePieceShoot(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ParallelDeadlineGroup(
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveResetPose(0.4, 2.65, 0, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterAmpNoteRed.value], driveTrain, log)
          ), 
          new SequentialCommandGroup(
            new DriveResetPose(0.4, 5.55, 0, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterAmpNoteBlue.value], driveTrain, log)
          ), 
          () -> alliance.getAlliance() == Alliance.Red
        ).andThen( new WaitUntilCommand( feeder::isPiecePresent ).withTimeout(0.5) ),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log)  
      ),
      new SetShooterWristSpeaker(WristAngle.ampCloseNoteShot, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(false, shooter, feeder, wrist, robotState, log),
      new ParallelDeadlineGroup(
        new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpNoteToCenterNoteRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpNoteToCenterNoteBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        ),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log)
      ).andThen( new WaitUntilCommand( feeder::isPiecePresent ).withTimeout(0.5) ),
      new SetShooterWristSpeaker(WristAngle.centerCloseNoteShot, 
          ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(false, shooter, feeder, wrist, robotState, log),
      new WristSetAngle(WristAngle.lowerLimit, wrist, log),
      new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromCenterNoteToEndCenterAutoRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromCenterNoteToEndCenterAutoBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
      )
    );
  }
}
