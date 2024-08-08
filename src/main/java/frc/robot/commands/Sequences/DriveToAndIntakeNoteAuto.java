// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.WristSetAngle;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.TrajectoryCache;
import frc.robot.utilities.TrajectoryCache.TrajectoryType;
import frc.robot.utilities.AllianceSelection;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAndIntakeNoteAuto extends ParallelDeadlineGroup {
  /** Creates a new DriveToAndIntakeNoteAuto. */
  public DriveToAndIntakeNoteAuto(TrajectoryType trajectoryRed, TrajectoryType trajectoryBlue, WristAngle wristAngle, DriveTrain drivetrain, Feeder feeder, Shooter shooter, Wrist wrist, Intake intake, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add the deadline command in the super() call. 
    // Add other commands using addCommands().
    super(
      new ConditionalCommand(
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[trajectoryRed.value], drivetrain, log),
        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[trajectoryBlue.value], drivetrain, log),
        () -> alliance.getAlliance() == Alliance.Red
      ).andThen( new WaitUntilCommand( () -> feeder.getFeederSetPercent() == 0.0).withTimeout(0.5) ) 
    );
    addCommands(
      new WristSetAngle(WristAngle.lowerLimit, wrist, log),
      new IntakePieceAuto(intake, feeder, robotState, log)
    );
  }
}
