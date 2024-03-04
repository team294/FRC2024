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
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Feeder;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakePiece extends SequentialCommandGroup {
  /** Creates a new IntakePiece. */
  public IntakePiece(Intake intake, Feeder feeder, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    addCommands(
      new IntakeSetPercent(IntakeConstants.intakePercent,IntakeConstants.centeringPercent, intake, log),
      new FeederSetPercent(FeederConstants.feederPercent, feeder, log),
      new WaitCommand(10).until(() -> feeder.isPiecePresent()),
      new IntakeSetPercent(0, 0, intake, log),
      new FeederSetPercent(0, feeder, log)
    );
  }
}
