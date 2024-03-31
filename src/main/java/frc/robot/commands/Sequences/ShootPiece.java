// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.commands.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class ShootPiece extends SequentialCommandGroup {

  /**
   * Shoots a piece using fixed shooter velocity.
   * @param velocityTop top shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param velocityBottom bottom shooter wheel velocity, in rpm  (+ = shoot forward, - = backwards)
   * @param waitForSpinDown true = wait for shooter motors to stop before returning.  False = return immediately after shooting, with shooter motors set to slow reverse speed.
   * @param shooter
   * @param feeder
   * @param wrist (not a required subsystem -- only reads the arm angle)
   * @param robotState
   * @param log
   */
  public ShootPiece(double velocityTop, double velocityBottom, boolean waitForSpinDown, Shooter shooter, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new RobotStateSet(BCRRobotState.State.SHOOTING, robotState, log),
        new ShooterSetVelocity(velocityTop, velocityBottom, VelocityType.waitForVelocity, shooter, log),
        new WaitUntilCommand( () -> !wrist.isEncoderCalibrated() || (Math.abs(wrist.getCurrentWristTarget() - wrist.getWristAngle()) < WristConstants.wristShootTolerance) )
      ).withTimeout(1.5),
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new WaitUntilCommand(()-> !feeder.isPiecePresent()).withTimeout(.4),
      new WaitCommand(.1),
      new ParallelCommandGroup(
        new ShooterSetPercent(ShooterConstants.shooterPercentStopQuickly, shooter, log),
        new FeederSetPercent(0, feeder, log),
        new RobotStateSetIdle(robotState, feeder, log)
      )
    );

    if (waitForSpinDown) {
      addCommands(
        new WaitCommand(0.5),
        new ShooterFeederStop(shooter, feeder, log)
      );
    }
  }

  /**
   * Shoots a piece using standard shooter velocities.
   * @param waitForSpinDown true = wait for shooter motors to stop before returning.  False = return immediately after shooting, with shooter motors set to slow reverse speed.
   * @param shooter
   * @param feeder
   * @param wrist (not a required subsystem -- only reads the arm angle)
   * @param robotState
   * @param log
   */
  public ShootPiece(boolean waitForSpinDown, Shooter shooter, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    this(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, waitForSpinDown, shooter, feeder, wrist, robotState, log);
  }

}
