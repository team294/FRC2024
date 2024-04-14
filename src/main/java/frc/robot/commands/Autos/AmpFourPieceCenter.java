// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.proto.Trajectory;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.IntakeConstants;
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
public class AmpFourPieceCenter extends SequentialCommandGroup {
  /** Creates a new SourceTwoPieceShoot. */
  public AmpFourPieceCenter(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
        // shoots
        new SetShooterWristSpeakerAuto(WristAngle.speakerShotFromSpeaker, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),

        // leaves speaker from amp side to outside of notes and pick up first note
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new SequentialCommandGroup(
                    new DriveResetPose(0.8, 1.6296, -60, false, driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpToFarCenterRed.value], driveTrain, log) 
                ),
                new SequentialCommandGroup(
                    new DriveResetPose(0.8, 6.6, 60, false, driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpToFarCenterBlue.value], driveTrain, log) 
                ),
                () -> alliance.getAlliance() == Alliance.Red
            ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
            new WristSetAngle(WristAngle.lowerLimit, wrist, log),
            new IntakePieceAuto(intake, feeder, robotState, log)
        ),

        new ConditionalCommand(
            new SequentialCommandGroup(
                // drives back to shoot 
                new ParallelCommandGroup(
                    new ConditionalCommand(
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFarCenterNoteToPodiumShotRed.value], driveTrain, log),
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFarCenterNoteToPodiumShotBlue.value], driveTrain, log),
                            () -> alliance.getAlliance() == Alliance.Red
                    ),
                    new SetShooterWristSpeakerAuto(WristAngle.ampFourPieceShot, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
                ),
                // shoots in speaker   
                new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
                // goes to next center note
                new ParallelDeadlineGroup(
                new ConditionalCommand(
                        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.drivePodiumShotToNextCenterNoteRed.value], driveTrain, log),
                        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.drivePodiumShotToNextCenterNoteBlue.value], driveTrain, log),
                        () -> alliance.getAlliance() == Alliance.Red
                        ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
                new WristSetAngle(WristAngle.lowerLimit, wrist, log),
                new IntakePieceAuto(intake, feeder, robotState, log)
                )
            ),
            // We missed the first note, to go to 2nd note
            new ParallelDeadlineGroup(
                new ConditionalCommand(
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFirstCenterAmpToNextCenterNoteRed.value], driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFirstCenterAmpToNextCenterNoteBlue.value], driveTrain, log),
                    () -> alliance.getAlliance() == Alliance.Red
                ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
                new WristSetAngle(WristAngle.lowerLimit, wrist, log),
                new IntakePieceAuto(intake, feeder, robotState, log)
            ),
            () -> (feeder.isPiecePresent() == true || intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold)
        ),

        new ConditionalCommand(
            new SequentialCommandGroup(
                // drive back to shoot note
                new ParallelCommandGroup(
                    new ConditionalCommand(
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveNextCenterNotetoPodiumShotRed.value], driveTrain, log),
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveNextCenterNotetoPodiumShotBlue.value], driveTrain, log),
                            () -> alliance.getAlliance() == Alliance.Red
                    ),
                new SetShooterWristSpeakerAuto(WristAngle.ampFourPieceShot, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
                ),
                // shoots piece
                new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
                // drives back to grab middle center note
                new ParallelDeadlineGroup(
                    new ConditionalCommand(
                        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.drivePodiumShotToCenterNoteRed.value], driveTrain, log),
                        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.drivePodiumShotToCenterNoteBlue.value], driveTrain, log),
                        () -> alliance.getAlliance() == Alliance.Red
                    ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
                    new WristSetAngle(WristAngle.lowerLimit, wrist, log),
                    new IntakePieceAuto(intake, feeder, robotState, log)
                ) 
            ),
            new ParallelDeadlineGroup(
                new ConditionalCommand(
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveNextCenterNoteToCenterNoteRed.value], driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveNextCenterNoteToCenterNoteBlue.value], driveTrain, log),
                    () -> alliance.getAlliance() == Alliance.Red
                ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
                new WristSetAngle(WristAngle.lowerLimit, wrist, log),
                new IntakePieceAuto(intake, feeder, robotState, log)
            ),
            () -> (feeder.isPiecePresent() == true || intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold)
        ),
        new ConditionalCommand(
            new SequentialCommandGroup(
                // drive back under stage to shoot note
                new ParallelCommandGroup(
                    new ConditionalCommand(
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNotetoPodiumShotRed.value], driveTrain, log),
                            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNotetoPodiumShotBlue.value], driveTrain, log),
                            () -> alliance.getAlliance() == Alliance.Red
                    ),
                    new SetShooterWristSpeakerAuto(WristAngle.endAmpFourcePieceShot, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
                ),
                // shoots note
                new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
            ),
            new SequentialCommandGroup(
                new WristSetAngle(WristAngle.lowerLimit, wrist, log)
            ),  
            () -> (feeder.isPiecePresent() == true || intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold)
        )
    );
  }
}