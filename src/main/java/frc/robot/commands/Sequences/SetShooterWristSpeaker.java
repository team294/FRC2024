// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.SpeakerModeSet;
import frc.robot.commands.FarShotSet;
import frc.robot.commands.IntakeStop;
import frc.robot.commands.RobotStateSetIdle;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterWristSpeaker extends SequentialCommandGroup {

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
  public SetShooterWristSpeaker(WristAngle angle, double velocityTop, double velocityBottom, 
    Shooter shooter, Wrist wrist, Intake intake, Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ParallelCommandGroup(
          new IntakeStop(intake, log),
          new WristSetAngle(angle, wrist, log),
          new ShooterSetVelocity(velocityTop, velocityBottom, VelocityType.waitForVelocity, shooter, log),
          new SpeakerModeSet(true, robotState, log),
          new FarShotSet(false, robotState, log),
          new RobotStateSetIdle(robotState, feeder, log)
        )
      );
  }
}
