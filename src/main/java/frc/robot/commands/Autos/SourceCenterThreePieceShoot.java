// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.Sequences.IntakePieceAuto;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceCenterThreePieceShoot extends SourceTwoPieceShoot {
  /** Creates a new SourceCenterThreePieceShoot. */
  public SourceCenterThreePieceShoot(Intake intake, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    super(intake, shooter, driveTrain, feeder, robotState, cache, alliance, log);
    addCommands(
       new ShootPiece(shooter, feeder, robotState, log),
      new ParallelCommandGroup(
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterAmpNoteRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveCenterAmpNoteBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpNoteToCenterStartRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromAmpNoteToCenterStartBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
      ),
      new ShootPiece(shooter, feeder, robotState, log),
      new ParallelCommandGroup(
        new IntakePieceAuto(intake, feeder, robotState, log),
        new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToCenterCloseNoteRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveToCenterCloseNoteBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
        )
      ),
      new ConditionalCommand(
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromCenterNoteToCenterStartRed.value], driveTrain, log), 
          new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveFromCenterNoteToCenterStartBlue.value], driveTrain, log), 
          () -> alliance.getAlliance() == Alliance.Red
      ),
      new ShootPiece(shooter, feeder, robotState, log)
    );
  }
}
