// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autos;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveToPose;
import frc.robot.commands.Sequences.SetShooterWristSpeaker;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceOneNoteShootLeave extends SequentialCommandGroup {
  /** Creates a new driveSourceOneNoteAuto. */
  public SourceOneNoteShootLeave(Intake intake, Wrist wrist, Shooter shooter, DriveTrain driveTrain, Feeder feeder, BCRRobotState robotState, AllianceSelection alliance, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
        ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log),
      new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, false, shooter, feeder, robotState, log),
      new ParallelCommandGroup(
        //new WristSetAngle(WristAngle.lowerLimit, wrist, log),
        new ConditionalCommand(
          new SequentialCommandGroup(
            new DriveResetPose(1.1, 3.463, 54, false, driveTrain, log),
            new DriveToPose(new Pose2d(7, 7.9, new Rotation2d(0)), driveTrain, log)
          ),
          new SequentialCommandGroup(
            new DriveResetPose(1.1, 4.6, -54, false, driveTrain, log),
            new DriveToPose(new Pose2d(7, 0.3, new Rotation2d(0)), driveTrain, log)
          ),
          () -> alliance.getAlliance() == Alliance.Red
        )
      )
    );
  }
}
