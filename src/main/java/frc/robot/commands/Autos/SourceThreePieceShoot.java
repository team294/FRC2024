// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
public class SourceThreePieceShoot extends SequentialCommandGroup {
  /** Creates a new CenterSourceThreePieceShoot. */
  public SourceThreePieceShoot(Intake intake, Shooter shooter, DriveTrain driveTrain, Feeder feeder, Wrist wrist, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToSourceCloseNoteRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToSourceCloseNoteBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, wrist, robotState, log),
      new ParallelCommandGroup(
        new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceNoteToFarNoteRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveSourceNoteToFarNoteBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
    );
  }
}
