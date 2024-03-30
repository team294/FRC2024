// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CenterLineLaunch extends SequentialCommandGroup {
  /** Creates a new CenterLineLaunch. */
  public CenterLineLaunch(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, feeder, robotState, log),
      new ShooterSetPercent(-0.02, shooter, log),
      new DriveResetPose(0.5, 6.4, 0.0, true, driveTrain, log),
      new DriveToPose(new Pose2d(0.5, 6.4, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log),
       new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 0.9, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)
          )
          , 
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 6.1, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 0.6, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)
          )
          , 
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 5.8, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new SetShooterFarShot(WristAngle.lowerLimit, ShooterConstans.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 4.1, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)
          )
          , 
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 4.1, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new SetShooterFarShot(WristAngle.lowerLimit, ShooterConstans.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)
       new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 4.1, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log)
          )
          , 
          new SequentialCommandGroup(
            new DriveToPose(new Pose2d(8.3, 2.4, new Rotation2d(0.0)), 2.0, SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
    )
        new SetShooterFarShot(WristAngle.lowerLimit, ShooterConstans.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log)

  }
}
