// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.FeederConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.StopType;
import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.TrajectoryConstants;
import frc.robot.Constants.WristConstants;
import frc.robot.Constants.WristConstants.WristAngle;
import frc.robot.commands.*;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Sequences.*;
import frc.robot.commands.ShooterSetVelocity.VelocityType;
import frc.robot.subsystems.*;
import frc.robot.utilities.*;
import frc.robot.utilities.BCRRobotState.State;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Define robot key utilities (DO THIS FIRST)
  private final FileLog log = new FileLog("A7");
  private final AllianceSelection allianceSelection = new AllianceSelection(log);

  // Define robot subsystems  
  private final DriveTrain driveTrain = new DriveTrain(allianceSelection, log);
  private final Intake intake = new Intake("Intake", log);
  private final Shooter shooter = new Shooter(log);
  private final Feeder feeder = new Feeder(log);
  private final Wrist wrist = new Wrist(log);

  // Define other utilities
  private final TrajectoryCache trajectoryCache = new TrajectoryCache(log);
  private final AutoSelection autoSelection = new AutoSelection(trajectoryCache, allianceSelection, log);
  private final BCRRobotState robotState = new BCRRobotState();
  
  // Is a subsystem, but requires a utility
  private final LED led = new LED(Constants.Ports.CANdle1, "LED", robotState, log, feeder);


  // Define controllers
  // private final Joystick xboxController = new Joystick(OIConstants.usbXboxController); //assuming usbxboxcontroller is int
  private final Joystick leftJoystick = new Joystick(OIConstants.usbLeftJoystick);
  private final Joystick rightJoystick = new Joystick(OIConstants.usbRightJoystick);
  private final Joystick coPanel = new Joystick(OIConstants.usbCoPanel);

  private final CommandXboxController xboxController = new CommandXboxController(OIConstants.usbXboxController);
  private boolean lastEnabledModeAuto = false;    // True if the last mode was auto

  // Set to this pattern when the robot is disabled
  
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    configureButtonBindings();
    configureShuffleboard();

    // driveTrain.setDefaultCommand(new DriveWithJoystick(leftJoystick, rightJoystick, driveTrain, log));
    driveTrain.setDefaultCommand(new DriveWithJoysticksAdvance(leftJoystick, rightJoystick, driveTrain, log));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    configureXboxButtons(); // configure xbox controller
    configureJoystickButtons(); // configure joysticks
    configureCopanel(); // configure copanel
    configureTriggers();
  }

  /**
   * Configures Shuffleboard for the robot
   */
  private void configureShuffleboard() {
    // Intake commands
    SmartDashboard.putData("Intake Set Percent", new IntakeSetPercent(intake, log));
    SmartDashboard.putData("Intake Stop", new IntakeStop(intake, log));

    // Shooter commands
    SmartDashboard.putData("Shooter Set Percent", new ShooterSetPercent(shooter, log));
    SmartDashboard.putData("Shooter Set Velocity", new ShooterSetVelocity(VelocityType.immediatelyEnd, shooter, log));
    SmartDashboard.putData("Shooter Calibration", new ShooterCalibrationRamp(shooter, log));
    SmartDashboard.putData("ShooterFeeder Stop", new ShooterFeederStop(shooter, feeder, log));

    // Feeder commands
    SmartDashboard.putData("Feeder Set Percent", new FeederSetPercent(feeder, log));
    SmartDashboard.putData("Feeder Stop", new FeederSetPercent(0.0, feeder, log));

    // Wrist commands
    SmartDashboard.putData("Wrist Set Percent", new WristSetPercentOutput(wrist, log));
    SmartDashboard.putData("Wrist Set Angle", new WristSetAngle(wrist, log));
    SmartDashboard.putData("Wrist Calibration", new WristCalibrationRamp(0.01, 0.4, wrist, log));
    SmartDashboard.putData("Wrist Stop", new WristSetPercentOutput(0.0, wrist, log));
  
    // Drive base commands
    SmartDashboard.putData("Drive Reset Pose", new DriveResetPose(driveTrain, log));
    SmartDashboard.putData("Drive To Pose", new DriveToPose(driveTrain, log));
    SmartDashboard.putData("Drive 6m +X", new DriveToPose(
      () -> driveTrain.getPose().plus(new Transform2d(6.0, 0.0, new Rotation2d(0.0))), 
      SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare, 
      TrajectoryConstants.maxPositionErrorMeters, TrajectoryConstants.maxThetaErrorDegrees, 
      false, false, driveTrain, log) );

    SmartDashboard.putData("Drive Calibration", new DriveCalibration(0.5, 5.0, 0.1, driveTrain, log));
    SmartDashboard.putData("Drive Turn Calibration", new DriveTurnCalibration(0.2, 5.0, 0.2 / 5.0, driveTrain, log));
    
    // SmartDashboard.putData("Test trajectory", new DriveTrajectory(CoordType.kRelative, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.test.value], driveTrain, log));
    SmartDashboard.putData("Source Start to near note",  new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.driveToSourceCloseNoteRed.value], driveTrain, log));
    SmartDashboard.putData("Drive to far note", new DriveTrajectory(CoordType.kAbsoluteResetPose, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.driveAmpNoteToFarNoteRed.value], driveTrain, log));

    SmartDashboard.putData("Drive Straight", new DriveStraight(false, false, false, driveTrain, log));

    // Sequences
    SmartDashboard.putData("Intake Piece", new IntakePiece(intake, feeder, wrist, robotState, log));
    SmartDashboard.putData("Shoot Piece", new ShootPiece(ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, feeder, robotState, log));
    SmartDashboard.putData("Stop All", new StopIntakeFeederShooter(intake, shooter, feeder, robotState, log));

    // Autos
    SmartDashboard.putData("Amp Three Piece Shoot", new AmpThreePieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Amp Two Piece Shoot", new AmpTwoPieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Center Two Piece Shoot", new CenterTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Three Piece Shoot", new SourceThreePieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Two Piece Shoot", new SourceTwoPieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));

    SmartDashboard.putData("Amp Source Three Piece Shoot", new AmpSourceThreePieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
    SmartDashboard.putData("Source Center Three Piece Shoot", new CenterThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));


    SmartDashboard.putData("Amp Source Three Piece Shoot", new AmpSourceThreePieceShoot(intake, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log));
  }

  /**
   * Creates triggers needed for the robot.
   */
  private void configureTriggers(){
    // Trigger to turn off intaking when a piece is detected in the feeder.
    // Note that this trigger will only turn off intaking if the robot is
    // currently in the INTAKE_TO_FEEDER state; otherwise, it does nothing.
    Trigger intakeStopTrigger = new Trigger(()-> DriverStation.isTeleopEnabled() && 
      robotState.getState() == State.INTAKING &&
      (feeder.isPiecePresent() ||  wrist.getWristAngle() > WristAngle.intakeLimit.value) );
    intakeStopTrigger.onTrue(
      new StopIntakingSequence(feeder, intake, robotState, log)
    );
  }

  /**
   * Configures XBox buttons and controls
   */
  private void configureXboxButtons(){
    //check povtrigger and axis trigger number bindings
    
    // Triggers for all xbox buttons
  
    Trigger xbLT = xboxController.leftTrigger();
    Trigger xbRT = xboxController.rightTrigger();
    Trigger xbA = xboxController.a();
    Trigger xbB = xboxController.b();
    Trigger xbY = xboxController.y();
    Trigger xbX = xboxController.x();
    Trigger xbLB = xboxController.leftBumper();
    Trigger xbRB = xboxController.rightBumper();
    Trigger xbBack = xboxController.back();
    Trigger xbStart = xboxController.start();
    Trigger xbPOVUp = xboxController.povUp();
    Trigger xbPOVRight = xboxController.povRight();
    Trigger xbPOVLeft = xboxController.povLeft();
    Trigger xbPOVDown = xboxController.povDown();

    // Prep for overhead speaker shot
    xbLB.onTrue(new SetShooterWristSpeaker(WristAngle.overheadShotAngle, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));

    // Move wrist down and then intake a piece
    xbRT.onTrue(new IntakePiece(intake, feeder, wrist, robotState, log));

    // Reverse the intake
    xbLT.onTrue(new IntakeSetPercent(-.3, -.3, intake, log));

    // Prep for at-speaker shot
    xbA.onTrue(new SetShooterWristSpeaker(WristAngle.speakerShotFromSpeaker, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));
    
    // Prep for podium speaker shot
    xbB.onTrue(new SetShooterWristSpeaker(WristAngle.speakerShotFromPodium, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));
    
    // Prep for mid-stage speaker shot
    xbY.onTrue(new SetShooterWristSpeaker(WristAngle.speakerShotFromMidStage, 
      ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, wrist, intake, feeder, robotState, log));

    // Prep for Far Shot
    xbPOVUp.onTrue(new SetShooterFarShot(WristAngle.speakerShotFromMidStage, ShooterConstants.shooterVelocityFarTop, ShooterConstants.shooterVelocityFarBottom, shooter, wrist, intake, feeder, robotState, log));
      new ParallelCommandGroup(
        new WristLowerSafe(WristAngle.lowerLimit, feeder, wrist, robotState, log),
        new SpeakerModeSet(true, robotState, log)
      );
    
    // Prep for pit shot when back button is pressed
    xbBack.onTrue(new SetShooterWristSpeaker(WristAngle.lowerLimit, 
      ShooterConstants.shooterVelocityPit, ShooterConstants.shooterVelocityPit, shooter, wrist, intake, feeder, robotState, log));
    // Shoot in slow speed pit shot when released
    xbBack.onFalse( new ShootPiece( ShooterConstants.shooterVelocityPit, ShooterConstants.shooterVelocityPit, 
      shooter, feeder, robotState, log) );

    // Prep for amp shot
    xbPOVRight.onTrue( new ParallelCommandGroup(
        new IntakeStop(intake, log),
        new WristSetAngle(WristAngle.ampShot, wrist, log),
        new SpeakerModeSet(false, robotState, log),
        new RobotStateSetIdle(robotState, feeder, log)
    ) );  

    // Stop all motors
    xbStart.onTrue(new ParallelCommandGroup(
        new IntakeStop(intake, log),
        new ShooterFeederStop(shooter, feeder, log),
        new RobotStateSetIdle(robotState, feeder, log)      
    ) );
    
  }

  /**
   * Define drivers joystick button mappings.
   */
  public void configureJoystickButtons() {
    JoystickButton[] left = new JoystickButton[3];
    JoystickButton[] right = new JoystickButton[3];

    for (int i = 1; i < left.length; i++) {
      left[i] = new JoystickButton(leftJoystick, i);
      right[i] = new JoystickButton(rightJoystick, i);
    }

    // Reset pose
    left[1].onTrue(new DriveResetPose( 0, false, driveTrain, log));

    // Shoot the note
    left[2].onTrue(
        new ConditionalCommand(
          new ShootPiece(ShooterConstants.shooterVelocityFarTop, ShooterConstants.shooterVelocityFarBottom, shooter, feeder, robotState, log)
          ,
          new ConditionalCommand(
            new ShootPiece( ShooterConstants.shooterVelocityTop, ShooterConstants.shooterVelocityBottom, shooter, feeder, robotState, log),
            new ShootPieceAmp(feeder, robotState, log),
            () -> robotState.isSpeakerMode()
          ),
          () -> robotState.isFarShootMode()
        )
        
    );

    // right[1].onTrue(new SetAimLock(true)); TODO implement this once vision is brought in
    // right[2] //Turn to face amp
    
    // right[1].onTrue(new ShootPiece(shooter, feeder, robotState, log));
    //right[2].onTrue(new IntakePiece(intake, feeder, robotState, log));
    
     
  }

  /** 
   * Define Copanel button mappings.
   *  
   *  1  3  5  8
   *  2  4  6  8
   *      
   *  9  11 13 7
   *  10 12 14 7
   * 
   *  15
   *  16
   */
  public void configureCopanel() {
    JoystickButton[] coP = new JoystickButton[20];

    for (int i = 1; i < coP.length; i++) {
      coP[i] = new JoystickButton(coPanel, i);
    }

    // top row UP then DOWN, from LEFT to RIGHT
    coP[1].onTrue(new WristSetAngle(WristAngle.climbStart, wrist, log));
    coP[3].onTrue(new SequentialCommandGroup(
      new WristSetPercentOutput(WristConstants.climbPercentOutput, wrist, log).until(() -> (wrist.getWristAngle() <= WristAngle.climbStop.value+5.0)),
      new WristSetAngle(WristAngle.climbStop, wrist, log)
    ));

  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoSelection.getAutoCommand(intake, wrist, shooter, feeder, driveTrain, trajectoryCache, robotState, log);
  }


  /**
   * Method called when robot is initialized.
   */
  public void robotInit() {
    SmartDashboard.putBoolean("RobotPrefs Initialized", RobotPreferences.prefsExist());
    if(!RobotPreferences.prefsExist()) {
      RobotPreferences.recordStickyFaults("RobotPreferences", log);
    }
    lastEnabledModeAuto = false;

    // compressor.disable();

    // Set initial robot position on field
    // This takes place a while after the drivetrain is created, so after any CanBus delays.
    driveTrain.resetPose(new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)));  
  }

  /**
   * robotPeriodic is run every 20msec
   */
  public void robotPeriodic(){
    log.advanceLogRotation();
    allianceSelection.periodic();
  }

  /**
   * Method called when robot is disabled.
   */
  public void disabledInit() {
    log.writeLogEcho(true, "Disabled", "Robot disabled");   // Don't log the word "Init" here -- it affects the Excel macro

    if (!lastEnabledModeAuto) {
      driveTrain.setDriveModeCoast(true);     // When pushing a disabled robot by hand, it is a lot easier to push in Coast mode!!!!
    }

    driveTrain.stopMotors();                // SAFETY:  Turn off any closed loop control that may be running, so the robot does not move when re-enabled.
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode

  }

  /**
   * Method called once every scheduler cycle when robot is disabled.
   */
  public void disabledPeriodic() {
    // Set robot state
    robotState.setState(State.IDLE);

    // Check for CAN bus error.  This is to prevent the issue that caused us to be eliminated in 2020!
    if (driveTrain.canBusError()) {
      RobotPreferences.recordStickyFaults("CAN Bus", log);
    }
    // boolean error = true;  
    // if (error == false) {
    //   if(!patternTeamMoving.isScheduled()) patternTeamMoving.schedule();
    // }
    // else {
    //   patternTeamMoving.cancel();
    //   led.setStrip("Red", 0.5, 0);
    // }
  }
  
  /**
   * Method called when auto mode is initialized/enabled.
   */
  public void autonomousInit() {
    log.writeLogEcho(true, "Auto", "Mode Init");
    lastEnabledModeAuto = true;

    driveTrain.setDriveModeCoast(false);

    // NOTE:  Do NOT reset the gyro or encoder here!!!!!
    // The first command in auto mode initializes before this code is run, and
    // it will read the gyro/encoder before the reset goes into effect.
  }

  /**
   * Method called once every scheduler cycle when auto mode is initialized/enabled
   */
  public void autonomousPeriodic() {
  }

  /**
   * Method called when teleop mode is initialized/enabled.
   */
  public void teleopInit() {
    log.writeLogEcho(true, "Teleop", "Mode Init");
    lastEnabledModeAuto = false;

    driveTrain.setDriveModeCoast(false);
    driveTrain.enableFastLogging(false);    // Turn off fast logging, in case it was left on from auto mode

    // Set robot state
    robotState.setState(State.IDLE);
  }

  /**
   * Method called once every scheduler cycle when teleop mode is initialized/enabled.
   */
  public void teleopPeriodic() {

  }
}