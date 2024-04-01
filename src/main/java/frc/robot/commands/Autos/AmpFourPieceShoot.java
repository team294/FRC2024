// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.Sequences.IntakePieceAuto;
import frc.robot.commands.Sequences.SetShooterWristSpeaker;
import frc.robot.commands.*;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.commands.Sequences.StopIntakingSequence;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpFourPieceShoot extends SequentialCommandGroup {
  /** Creates a new AmpTwoPieceShoot. */
  public AmpFourPieceShoot(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
      // ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ParallelCommandGroup(
          new ConditionalCommand(
              new SequentialCommandGroup(
                new DriveResetPose(0.2, 1.8, 0, false, driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromStraightAmpStartToNearAmpNoteRed.value], driveTrain, log)
              ),
              new SequentialCommandGroup(
                new DriveResetPose(0.2, 6.4, 0, false, driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromStraightAmpStartToNearAmpNoteBlue.value], driveTrain, log)
              ),
            () -> alliance.getAlliance() == Alliance.Red
          )
        ),
        new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
          ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, robotState, log),
        new ParallelDeadlineGroup(
          new ConditionalCommand(
              new DriveToPose(new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(-23))), driveTrain, log),
              new DriveToPose(new Pose2d(2.0, 6.95, new Rotation2d(Math.toRadians(23))), driveTrain, log),
            () -> alliance.getAlliance() == Alliance.Red
          ),
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log)
        ),
        new StopIntakingSequence(feeder, intake, robotState, log),
        new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
          ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, robotState, log),
        new ParallelDeadlineGroup(
          new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpNoteToFarNoteRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpNoteToFarNoteBlue.value], driveTrain, log),
            () -> alliance.getAlliance() == Alliance.Red
          ),
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log)
        ),
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new StopIntakingSequence(feeder, intake, robotState, log),
            new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
              ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
          ),
          new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpFarNoteToNearCenterNoteRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpFarNoteToNearCenterNoteBlue.value], driveTrain, log),
            () -> alliance.getAlliance() == Alliance.Red
          )
        ),
        new StopIntakingSequence(feeder, intake, robotState, log),
        new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
          ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, robotState, log),
        new ParallelDeadlineGroup(
          new ConditionalCommand(
              new DriveToPose(new Pose2d(2.0, 2.65, new Rotation2d(Math.toRadians(0))), driveTrain, log),
              new DriveToPose(new Pose2d(2.0, 5.55, new Rotation2d(Math.toRadians(0))), driveTrain, log),
            () -> alliance.getAlliance() == Alliance.Red
          ),
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log)
        ),
        new StopIntakingSequence(feeder, intake, robotState, log),
        new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
          ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, robotState, log),
        new ParallelCommandGroup(
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log),
          new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNoteToCenterFieldPieceRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNoteToCenterFieldPieceBlue.value], driveTrain, log),
            () -> alliance.getAlliance() == Alliance.Red
          )
        )
    );
  }
}
