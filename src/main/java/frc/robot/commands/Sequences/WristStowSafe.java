// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.IntakeSetPercent;
import frc.robot.commands.RobotStateSet;
import frc.robot.commands.WristSetAngle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristStowSafe extends SequentialCommandGroup {
  /** Creates a new WristStowSafe. */
  public WristStowSafe(Intake intake, Feeder feeder, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new SequentialCommandGroup(
          new RobotStateSet(BCRRobotState.State.INTAKING, robotState, log),
          new FeederSetPercent(FeederConstants.feederPercent, feeder, log)
        ),
        new WaitCommand(0),
        () -> (!feeder.isPiecePresent())
      ),
      new WristSetAngle(WristAngle.lowerLimit, wrist, log)
    );
  }
}
