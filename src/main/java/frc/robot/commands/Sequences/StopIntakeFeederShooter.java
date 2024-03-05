// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState;

public class StopIntakeFeederShooter extends SequentialCommandGroup {

  /**
   * Stops the intake, feeder, and shooter motors.
   * Resets the robot state to either IDLE_WITH_PIECE or IDLE_NO_PIECE,
   * using the feeder sensor to decide if it has a piece.
   * @param intake
   * @param shooter
   * @param feeder
   * @param robotState
   * @param log
   */
  public StopIntakeFeederShooter(Intake intake, Shooter shooter, Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeStop(intake, log),
      new ShooterFeederStop(shooter, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
