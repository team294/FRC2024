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
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.Sequences.IntakePieceAuto;
import frc.robot.commands.Sequences.SetShooterWristSpeakerAuto;
import frc.robot.commands.Sequences.ShootPiece;
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
public class SourceOnePieceDriveToFifthNote extends SequentialCommandGroup {
  /** Creates a new SourceOnePieceAuto. */
  public SourceOnePieceDriveToFifthNote(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristSpeakerAuto(WristAngle.speakerShotFromSpeaker, ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ParallelDeadlineGroup(
        new ConditionalCommand(
          new SequentialCommandGroup(
              new DriveResetPose(0.8, 3.7296, 60, false, driveTrain, log),
              new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromSourceToSideMobilityRed.value], driveTrain, log) 
          ),
          new SequentialCommandGroup(
              new DriveResetPose(0.8, 4.5, -60, false, driveTrain, log),
              new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromSourceToSideMobilityBlue.value], driveTrain, log) 
          ),
          () -> alliance.getAlliance() == Alliance.Red
        ).andThen( new WaitUntilCommand( () -> feeder.getFeederSetPercent() == 0.0 ).withTimeout(0.5) ),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log)
      ),
      new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.0, 7.467, new Rotation2d(0)), driveTrain, log)
          ),
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.0, 0.7696, new Rotation2d(0)), driveTrain, log)
          ),
          () -> alliance.getAlliance() == Alliance.Red
      )
    );
  }
}
