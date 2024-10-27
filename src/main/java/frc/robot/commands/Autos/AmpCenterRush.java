// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.Sequences.DriveBackAndSetWristAuto;
import frc.robot.commands.Sequences.DriveToAndIntakeNoteAuto;
import frc.robot.commands.Sequences.IntakePieceAuto;
import frc.robot.commands.Sequences.ScoreNoteAuto;
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
import frc.robot.utilities.TrajectoryCache.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AmpCenterRush extends SequentialCommandGroup {
  /** Creates a new AmpCenterRush. */
  public AmpCenterRush(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, TrajectoryCache cache, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(

        new SequentialCommandGroup(

          new WaitCommand(1),
          new ShootPiece(ShooterConstants.shooterVelocityPit, ShooterConstants.shooterVelocityPit, false, shooter, feeder, wrist, robotState, log),
          //new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new IntakePieceAuto(intake, feeder, robotState, log)

        ),

        new DriveTrajectory(CoordType.kAbsolute, StopType.kBrake, cache.cache[TrajectoryType.driveAmpRushtoFarRight.value], driveTrain, alliance, log)
        //new DriveToAndIntakeNoteAuto(new Pose2d(0.77,1.1296, Rotation2d.fromDegrees(0)), new Pose2d(0.77, 7.1, Rotation2d.fromDegrees(0)), TrajectoryType.driveAmpRushtoFarRight, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log)

      ),

      // drives back to shoot (Changed from a ParallelCommandGroup to a ParallelDeadlineGroup in command)
      new DriveBackAndSetWristAuto(TrajectoryType.driveFarCenterNoteToPodiumShot, WristAngle.ampFourPieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      // shoots in speaker   
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
      
      // goes to dropped note
      new DriveToAndIntakeNoteAuto(TrajectoryType.drivePodiumShotToDroppedNote, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),

      // drive back to shoot note (Changed from a ParallelCommandGroup to a ParallelDeadlineGroup in command)
      new DriveBackAndSetWristAuto(TrajectoryType.driveDroppedNoteToPodiumShot, WristAngle.ampFourPieceShot, driveTrain, feeder, shooter, wrist, intake, robotState, cache, alliance, log),
      
      // shoots piece
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log)
    );
  }
}
