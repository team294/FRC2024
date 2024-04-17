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
public class SourceThreeNoteCenter extends SequentialCommandGroup {
  /** Creates a new SourceTwoPieceShoot. */
  public SourceThreeNoteCenter(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());


    addCommands(
        // shoots
        new SetShooterWristSpeakerAuto(WristAngle.speakerShotFromSide, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),

        // leaves speaker from source side to outside of notes
        new ConditionalCommand(
            new SequentialCommandGroup(
                new DriveResetPose(0.8, 3.73, 54, false, driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotesRed.value], driveTrain, log) 
            ),
            new SequentialCommandGroup(
               new DriveResetPose(0.8, 4.5, -54, false, driveTrain, log),
               new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotesBlue.value], driveTrain, log) 
            ),
            () -> alliance.getAlliance() == Alliance.Red
        ),
        
        // goes under stage to intake up middle center note
        new ParallelDeadlineGroup(
           new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotestoCenterNoteRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotestoCenterNoteBlue.value], driveTrain, log),
                () -> alliance.getAlliance() == Alliance.Red
                ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log)
        ),

        // drives back to shoot in speaker
        new ParallelCommandGroup(
            new ConditionalCommand(
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNotetoOutsideStageRed.value], driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNotetoOutsideStageBlue.value], driveTrain, log),
                    () -> alliance.getAlliance() == Alliance.Red
            ),
            new SetShooterWristSpeakerAuto(WristAngle.sourceThreePieceShot, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
        ),

        // shoots in speaker   
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        
        // drives back through under stage to grab left of middle center note
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNoteRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNoteBlue.value], driveTrain, log),
                () -> alliance.getAlliance() == Alliance.Red
            ).andThen( new WaitUntilCommand( () -> feeder.isPiecePresent() && feeder.getFeederSetPercent() >= 0.0 ).withTimeout(0.5) ),
            new WristSetAngle(WristAngle.lowerLimit, wrist, log),
            new IntakePieceAuto(intake, feeder, robotState, log)
        ),
       
        // drive back under stage to shoot note
        new ParallelCommandGroup(
            new ConditionalCommand(
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveLeftCenterNotetoOutsideStageRed.value], driveTrain, log),
                    new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveLeftCenterNotetoOutsideStageBlue.value], driveTrain, log),
                    () -> alliance.getAlliance() == Alliance.Red
            ),
            new SetShooterWristSpeakerAuto(WristAngle.sourceThreePieceShot, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
        ),
        // shoots note
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        
        //leaves to midfield to get headstart in teleop
        new ParallelDeadlineGroup(
            new ConditionalCommand(
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNoteRed.value], driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNoteBlue.value], driveTrain, log),
                () -> alliance.getAlliance() == Alliance.Red
            ),
            new WristSetAngle(WristAngle.lowerLimit, wrist, log)
        )      
    );
  }
}