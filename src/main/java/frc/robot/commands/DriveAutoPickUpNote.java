// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.JeVoisCamera;
import frc.robot.Constants.VisionConstants.JeVoisConstants;
import frc.robot.utilities.FileLog;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class DriveAutoPickUpNote extends PIDCommand {
  private int toleranceCounter;

  /** Creates a new DriveAutoAim. */
  public DriveAutoPickUpNote(JeVoisCamera camera, FileLog log) {
    super(
        // The controller that the command will use
        new PIDController(0, 0, 0),
        // This should return the measurement
        () -> 0,
        // This should return the setpoint (can also be a constant)
        () -> JeVoisConstants.width / 2,
        // This uses the output
        output -> {
          // Use the output here
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
  }

  @Override
  public void initialize() {
    super.initialize();
    toleranceCounter = 0;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
  //   if (!camera.encoderCalibrated() ||         // End immediately if encoder can't read
  //     Math.abs(elevator.getElevatorPos() - target) <= 0.5) {
  //       toleranceCounter++;
  //       log.writeLog(false, "ElevatorSetPosition", "Within Tolerance", "Target Position", target, "Position", elevator.getElevatorPos(), "Tol count", toleranceCounter);
  //   }
  //   return (toleranceCounter > 5);
    return false;
  }
}
