// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.WristSetPercentOutput;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbEnd extends SequentialCommandGroup {
  /** Creates a new ClimbEnd. */
  public ClimbEnd(Wrist wrist, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new WristSetPercentOutput(WristConstants.climbPercentOutput, wrist, log).until(() -> (wrist.getWristAngle() <= WristAngle.climbStop.value + 5.0)),
      new WristSetAngle(WristAngle.climbStop, wrist, log)
      // new CANdleRainbowAnimation(led, LEDSegmentRange.StripHorizontal)
    );
  }
}
