// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.Constants.FeederConstants;
import frc.robot.commands.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

public class IntakePiece extends SequentialCommandGroup {

  /**
   * Moves the wrist down, then turns on the intake and feeder motors, and sets the robot state
   * to INTAKE_TO_FEEDER (but only if we don't have a piece).
   * <p><b>Note:</b> This sequence does not stop intaking!  This requires a 
   * trigger to turn off intaking when a piece gets to the feeder, or the
   * driver can manually turn off intaking.
   * @param intake
   * @param feeder
   * @param wrist
   * @param robotState
   * @param log
   */
  public IntakePiece(Intake intake, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new WristSetAngle(WristAngle.lowerLimit, wrist, log),
          new RobotStateSet(BCRRobotState.State.INTAKING, robotState, log),
          new IntakeSetPercent(IntakeConstants.intakePercent,IntakeConstants.centeringPercent, intake, log),
          new FeederSetPercent(FeederConstants.feederPercent, feeder, log)
        ),
        new WaitCommand(0),
        () -> (!feeder.isPiecePresent())
      )
    );
  }
}
