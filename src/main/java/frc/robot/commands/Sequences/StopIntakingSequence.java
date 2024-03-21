// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.*;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

public class StopIntakingSequence extends ParallelCommandGroup {
  
  /**
   * Stops the intaking sequence and sets the robot state to IDLE
   * @param feeder Feeder subsystem
   * @param intake Intake subsystem
   * @param robotState Object with current robot state
   * @param log
   */
  public StopIntakingSequence(Feeder feeder, Intake intake, BCRRobotState robotState, FileLog log) {
    addCommands(
      new IntakeSetPercent(0, 0, intake, log),
      new FeederSetPercent(0, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
