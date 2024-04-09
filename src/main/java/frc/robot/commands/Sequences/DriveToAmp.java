// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Wrist;
import frc.robot.utilities.AllianceSelection;
import frc.robot.utilities.BCRRobotState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.BCRRobotState.ShotMode;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveToAmp extends SequentialCommandGroup {
  /** Creates a new DriveToAmp. */
  public DriveToAmp(AllianceSelection allianceSelection, Intake intake, Feeder feeder, Wrist wrist,DriveTrain driveTrain, BCRRobotState robotState, FileLog log) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new DriveToPose(allianceSelection.getAmpPos(), driveTrain, log),
        new IntakeStop(intake, log),
        new WristSetAngle(WristAngle.ampShot, wrist, log),
        new SpeakerModeSet(false, robotState, log),
        new ShotModeSet(ShotMode.STANDARD, robotState, log),
        new RobotStateSetIdle(robotState, feeder, log)
      ) 
    );
  }
}
