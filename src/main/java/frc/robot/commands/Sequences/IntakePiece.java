// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.IntakeSetPercent;
import frc.robot.commands.RobotStateSetIdle;
import frc.robot.subsystems.Intake;
import frc.robot.utilities.BCRRobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class IntakePiece extends SequentialCommandGroup {

  /**
   * Intakes a piece.  Stops after the feeder sensor detects a piece, or 
   * after 10 seconds if no piece is found.
   * @param intake
   * @param feeder
   * @param robotState
   * @param log
   */
  public IntakePiece(Intake intake, Feeder feeder, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new IntakeSetPercent(IntakeConstants.intakePercent,IntakeConstants.centeringPercent, intake, log),
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new WaitCommand(10).until(() -> feeder.isPiecePresent()),
      new IntakeSetPercent(0, 0, intake, log),
      new FeederSetPercent(0, feeder, log),
      new RobotStateSetIdle(robotState, feeder, log)
    );
  }
}
