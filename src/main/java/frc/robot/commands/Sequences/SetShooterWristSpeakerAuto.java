// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.BCRRobotState.ShotMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterWristSpeakerAuto extends SequentialCommandGroup {

  /**
   * Sets Shooter and Wrist to prime for a speaker shot.  Also stops the intake and sets the robot state.
   * @param angle wrist target angle for shot, in degrees (+ = up, -  = down, 0 = horizontal)
   * @param velocityTop top shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param velocityBottom bottom shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param shooter
   * @param wrist
   * @param intake
   * @param feeder
   * @param robotState
   * @param log
   */
  public SetShooterWristSpeakerAuto(WristAngle angle, double velocityTop, double velocityBottom, 
    Shooter shooter, Wrist wrist, Intake intake, Feeder feeder, BCRRobotState robotState, FileLog log) {
    addCommands(
      // If we don't detect a piece, but the intake current is high, then assume that the piece is still
      // in the the process of intaking.  So, finish the intake sequence before changing the wrist angle
      // (which would be an issue for the piece getting into the wrist) and before setting the shooter motor
      // speed (which would cause an errant shot).
      // Note that the anti-jam code in wrist.periodic() is still active and will turn off the intake motor if needed after 0.5 sec.
      new ConditionalCommand(
        new SequentialCommandGroup(
          new FileLogWrite(false, false, "SetShooterWristSpeakerAuto", "Finish intaking", log, "Intake current", intake.getIntakeAmps()),
          new WaitCommand(1.0).until(() -> feeder.isPiecePresent()),
          new IntakeSetPercent(0, 0, intake, log),
          new FeederSetPercent(-0.05, feeder, log),
          new WaitCommand(0.1),
          new FeederSetPercent(0.0, feeder, log),      
          new RobotStateSetIdle(robotState, feeder, log)
        ), 
        new WaitCommand(0.01), 
        () -> !feeder.isPiecePresent() && intake.getIntakeAmps() >= IntakeConstants.intakingPieceCurrentThreshold
      ),

      // Turn off intake, set wrist angle, set shooter speed
      new ParallelCommandGroup(
        new IntakeStop(intake, log),
        new FeederSetPercent(0.0, feeder, log),     // Added for autos
        new WristSetAngle(angle, wrist, log),
        new ShooterSetVelocity(velocityTop, velocityBottom, VelocityType.waitForVelocity, shooter, log).withTimeout(0.5),  // Added timeout for auto
        new SpeakerModeSet(true, robotState, log),
        new ShotModeSet(ShotMode.STANDARD, robotState, log),
        new RobotStateSetIdle(robotState, feeder, log)
      )
    );
  }
}
