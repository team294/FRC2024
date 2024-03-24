// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.BCRRobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;

public class ShootPiece extends SequentialCommandGroup {

  /**
   * Shoots a piece using fixed shooter velocity.
   * @param velocityTop top shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param velocityBottom bottom shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param shooter
   * @param feeder
   * @param robotState
   * @param log
   */
  public ShootPiece(double velocityTop, double velocityBottom, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new ShooterSetVelocity(ShooterConstants.shooterVelocity, VelocityType.waitForVelocity, shooter, log),
      new RobotStateSet(BCRRobotState.State.SHOOTING, robotState, log),
      new ShooterSetVelocity(velocityTop, velocityBottom, VelocityType.waitForVelocity, shooter, log).withTimeout(1.5),
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new WaitCommand(.5),
      new ShooterFeederStop(shooter, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }

  /**
   * Shoots a piece using standard shooter velocities.
   * @param shooter
   * @param feeder
   * @param robotState
   * @param log
   */
  public ShootPiece(Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log) {
    this(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, feeder, robotState, log);
  }

}
