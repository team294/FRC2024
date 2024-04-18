// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.BCRRobotState.ShotMode;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootFullSequence extends SequentialCommandGroup {
  /** Creates a new ShootFullSequence. */
  public ShootFullSequence(Shooter shooter, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
          new ConditionalCommand(
            // Shoot in speaeker
            new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, true, shooter, feeder, wrist, robotState, log),
            // Shoot in amp
            new ShootPieceAmp(feeder, robotState, log),
            () -> robotState.isSpeakerMode()
          ),
          new ConditionalCommand(
            // Short pass lobbing note towards alliance partner
            new ShootPiece(ShooterConstants.shooterVelocityShortPassTop, ShooterConstants.shooterVelocityShortPassBottom, true, shooter, feeder, wrist, robotState, log), 
            // Far Pass lobbing note over stage to alliance partner
            new ShootPiece(ShooterConstants.shooterVelocityFarPassTop, ShooterConstants.shooterVelocityFarPassBottom, true, shooter, feeder, wrist, robotState, log),
            () -> robotState.getShotMode() == ShotMode.SHORT_PASS
            ),
          () -> robotState.getShotMode() == ShotMode.STANDARD
        )
    );
  }
}
