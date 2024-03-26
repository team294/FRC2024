// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.FeederSetPercent;
import frc.robot.commands.WristSetAngle;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class WristLowerSafe extends SequentialCommandGroup {
  /**
   * Safely lowers wrist
   * @param angle target angle to move wrist to WristAngle (see Constants)
   * @param feeder feeder
   * @param wrist wrist
   * @param robotState state
   * @param log log
   */
  public WristLowerSafe(WristAngle angle, Feeder feeder, Wrist wrist, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ConditionalCommand(
        new ParallelCommandGroup(
          new SequentialCommandGroup(
            new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
            new WaitCommand(4).until(()->feeder.isPiecePresent()),
            new ConditionalCommand(
              new SequentialCommandGroup(   // Back off piece slightly from shooter wheels
                new FeederSetPercent(FeederConstants.feederBackPiecePercent, feeder, log),
                new WaitCommand(FeederConstants.feederBackPieceTime),
                new FeederSetPercent(0.0, feeder, log)
              ),
              new FeederSetPercent(0, feeder, log),
              ()->feeder.isPiecePresent()
            )
          ),
          new WristSetAngle(angle, wrist, log)
        ),
        new WristSetAngle(angle, wrist, log),
        () -> (!feeder.isPiecePresent() && angle.value <= WristAngle.clearBellyPanMinAngle.value)
      )
    );
  }
}
