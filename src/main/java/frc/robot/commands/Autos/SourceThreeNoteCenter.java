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
    // need to fix just for now ???????????!!!!!!!!!!!!!!!!!!!!!!   efhuqefhqfhuoquofqouefhq
    // idk what to fix this is jank

    addCommands(
        // leaves speaker from source side to outside of notes
        new SequentialCommandGroup(
            new DriveResetPose(1.1, 3.463, 54, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotesRed.value], driveTrain, log) 
        ),
        // shoots preloaded note
        new WristSetAngleWithVision(wrist, alliance, driveTrain, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        // goes under stage to intake up middle center note
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveResetPose(2.72, 5.3296, 45, false, driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceOutsideNotesRedtoCenterNote.value], driveTrain, log)
            ),
            new WristSetAngle(WristAngle.lowerLimit, wrist, log),
            new IntakePieceAuto(intake, feeder, robotState, log)
        ),
        // drives back to shoot in speaker
        new SequentialCommandGroup(
            new DriveResetPose(8.1742, 4.1546, 0, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterNotetoOutsideStage.value], driveTrain, log)
            ),    
        // shoots in speaker   
        new WristSetAngleWithVision(wrist, alliance, driveTrain, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        // drives back through under stage to grab left of middle center note
        new ParallelDeadlineGroup(
            new SequentialCommandGroup(
                new DriveResetPose(3.7582, 2.7516, 0, false, driveTrain, log),
                new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNote.value], driveTrain, log)
            ),
            new WristSetAngle(WristAngle.lowerLimit, wrist, log),
            new IntakePieceAuto(intake, feeder, robotState, log)
        ),
        // drive back under stage to shoot note
        new SequentialCommandGroup(
            new DriveResetPose(8.0652, 5.7986, 0, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveLeftCenterNotetoOutsideStage.value], driveTrain, log)
            ),
        // shoots note
        new WristSetAngleWithVision(wrist, alliance, driveTrain, log),
        new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
        //leaves to midfield to get headstart in teleop
        new SequentialCommandGroup(
            new DriveResetPose(3.7582, 2.7516, 0, false, driveTrain, log),
            new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveOutsideStageLeftCenterNote.value], driveTrain, log)
        )      
    );
  }
}