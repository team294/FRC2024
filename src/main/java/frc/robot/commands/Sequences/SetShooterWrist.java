// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.SetSpeakerMode;
import frc.robot.commands.ShooterSetVelocity;
import frc.robot.commands.WristSetAngle;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SetShooterWrist extends SequentialCommandGroup {
  /** Creates a new SetShooterWrist. */
  public SetShooterWrist(WristAngle angle, Shooter shooter, Wrist wrist, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new WristSetAngle(angle, wrist, log),
        new ShooterSetVelocity(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, VelocityType.waitForVelocity, shooter, log),
        new SetSpeakerMode(true, robotState, log)
      )
    );
  }
}
