// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.FileLog;
import frc.robot.commands.*;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootPiece extends SequentialCommandGroup {
  /** Creates a new ShootPiece. */
  public ShootPiece(Shooter shooter, Intake intake, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ShooterSetPercent(ShooterConstants.shooterPercent, shooter, log),
      new WaitCommand(1),
      new FeederSetPercent(ShooterConstants.feederPercent, shooter, log),
      new IntakeSetPercent(IntakeConstants.intakePercent, IntakeConstants.centeringPercent, intake,  log),
      new WaitCommand(1),
      new IntakeStop(intake, log),
      new ShooterFeederStop(shooter, log)
    );
  }
}
