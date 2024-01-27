// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CANcoderConfigurator;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants.SwerveConstants;
import frc.robot.Constants.DriveConstants.DrivingMotorPID;
import frc.robot.Constants.DriveConstants.TurningMotorPID;
import frc.robot.utilities.MathSwerveModuleState;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.MathBCR;
import frc.robot.utilities.Wait;

import static frc.robot.utilities.StringUtil.*;

public class SwerveModule {
      
  private final String swName;    // Name for this swerve module
  private final FileLog log;
  private final boolean driveMotorInverted;
  private final boolean turningMotorInverted;
  private final double turningOffsetDegrees;

  // Drive motor objects
  private final CANSparkMax driveMotor;
  private final SparkPIDController drivePID;
  private final RelativeEncoder driveEncoder;

  // Turning motor objects
  private final CANSparkMax turningMotor;
  private final SparkPIDController turningPID;
  private final RelativeEncoder turningEncoder;
  
  // CANCoder objects
  private final CANcoder turningCanCoder;
  private final CANcoderConfigurator turningCanCoderConfigurator;
  private CANcoderConfiguration turningCanCoderConfig;
	
  // CANCoder signals and sensors
	private final StatusSignal<Double> turningCanCoderPosition;			// CanCoder position, in CANCoder rotations
	private final StatusSignal<Double> turningCanCoderVelocity;			// Encoder Velocity, in CANCoder rotations/second

  // Variables for encoder zeroing
  private double driveEncoderZero = 0;      // Reference raw encoder reading for drive motor encoder.  Calibration sets this to zero.
  private double cancoderZero = 0;          // Reference raw encoder reading for CanCoder.  Calibration sets this to the absolute position from RobotPreferences.
  private double turningEncoderZero = 0;    // Reference raw encoder reading for turning motor encoder.  Calibration sets this to match the CanCoder.


  private final SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSDrive, SwerveConstants.kVDrive, SwerveConstants.kADrive);
  // private final SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(SwerveConstants.kSTurn, SwerveConstants.kVTurn);
  

  /**
   * Constructs a SwerveModule.
   *
   * @param swName The name of this swerve module, for use in Shuffleboard and logging
   * @param driveMotorAddress The CANbus address of the drive motor.
   * @param turningMotorAddress The CANbus address of the turning motor.
   * @param cancoderAddress The CANbus address of the turning encoder.
   * @param driveMotorInverted True = invert drive motor direction
   * @param turningMotorInverted True = invert turning motor direction
   * @param cancoderReverse True = reverse direction reading of CANcoder
   * @param turningOffsetDegrees Offset degrees in the turning motor to point to the 
   * front of the robot.  Value is the desired encoder zero point, in absolute magnet position reading.
   * @param log FileLog reference
   */
  public SwerveModule(String swName, int driveMotorAddress, int turningMotorAddress, int cancoderAddress,
      boolean driveMotorInverted, boolean turningMotorInverted, boolean cancoderReversed,
      double turningOffsetDegrees, FileLog log) {

    // Save the module name and logfile
    this.swName = swName;
    this.log = log;
    this.driveMotorInverted = driveMotorInverted;
    this.turningMotorInverted = turningMotorInverted;
    this.turningOffsetDegrees = turningOffsetDegrees;

    // Create motor and encoder objects
    driveMotor = new CANSparkMax(driveMotorAddress, CANSparkLowLevel.MotorType.kBrushless);
    drivePID = driveMotor.getPIDController();
    driveEncoder = driveMotor.getEncoder();

    turningMotor = new CANSparkMax(turningMotorAddress, CANSparkLowLevel.MotorType.kBrushless);
    turningPID = turningMotor.getPIDController();
    turningEncoder = turningMotor.getEncoder();

    turningCanCoder = new CANcoder(cancoderAddress);
    turningCanCoderConfigurator = turningCanCoder.getConfigurator();
    turningCanCoderPosition = turningCanCoder.getPosition();
    turningCanCoderVelocity = turningCanCoder.getVelocity();

    // Setup CANCoder configuration
    turningCanCoderConfig = new CANcoderConfiguration();			// Factory default configuration
    turningCanCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    turningCanCoderConfig.MagnetSensor.SensorDirection = cancoderReversed ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive;  //TODO Determine which direction is reversed

    // Configure the swerve module motors and encoders
    configSwerveModule();

    // other configs for drive and turning motors
    setMotorModeCoast(true);        // true on boot up, so robot is easy to push.  Change to false in autoinit or teleopinit

  }

  // ********** Swerve module configuration methods

  /**
   * Configures the motors and encoders, uses default values from the constructor.
   * In general, this should only be called from the constructor when the robot code is starting.
   * However, if the robot browns-out or otherwise partially resets, then this can be used to 
   * force the encoders to have the right calibration and settings, especially the
   * calibration angle for each swerve module.
   */
  public void configSwerveModule() {
    // configure drive motors
    driveMotor.restoreFactoryDefaults();
    driveMotor.setInverted(driveMotorInverted);
    driveMotor.setIdleMode(IdleMode.kCoast);
    driveMotor.setOpenLoopRampRate(0.00); //seconds from neutral to full
    driveMotor.setClosedLoopRampRate(0.00); //seconds from neutral to full
    driveMotor.setSmartCurrentLimit(80, 35);
    driveMotor.enableVoltageCompensation(SwerveConstants.voltageCompSaturation);

    // configure drive PID
    drivePID.setP(DrivingMotorPID.kP);
    drivePID.setI(DrivingMotorPID.kI);
    drivePID.setD(DrivingMotorPID.kD);
    drivePID.setIZone(DrivingMotorPID.kIz);
    drivePID.setFF(DrivingMotorPID.kFF);
    drivePID.setOutputRange(DrivingMotorPID.kMinOutput, DrivingMotorPID.kMaxOutput);

    // configure turning motors
    turningMotor.restoreFactoryDefaults();
    turningMotor.setInverted(turningMotorInverted);
    turningMotor.setIdleMode(IdleMode.kCoast);
    turningMotor.setOpenLoopRampRate(0.1); //seconds from neutral to full
    turningMotor.setClosedLoopRampRate(0.1); //seconds from neutral to full
    driveMotor.setSmartCurrentLimit(40, 25);
    driveMotor.enableVoltageCompensation(SwerveConstants.voltageCompSaturation);

    // configure turning PID
    turningPID.setP(TurningMotorPID.kP);
    turningPID.setI(TurningMotorPID.kI);
    turningPID.setD(TurningMotorPID.kD);
    turningPID.setIZone(TurningMotorPID.kIz);
    turningPID.setFF(TurningMotorPID.kFF);
    turningPID.setOutputRange(TurningMotorPID.kMinOutput, TurningMotorPID.kMaxOutput);

    // Apply configuration to the cancoder
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.
    turningCanCoderConfigurator.apply(turningCanCoderConfig);

    // NOTE!!! When the Cancoder or TalonFX encoder settings are changed above, then the next call to 
    // getCanCoderDegrees() getTurningEncoderDegrees() may contain an old value, not the value based on 
    // the updated configuration settings above!!!!  The CANBus runs asynchronously from this code, so 
    // sending the updated configuration to the CanCoder/Falcon and then receiving an updated position measurement back
    // may take longer than this code.
    // The timeouts in the configuration code above (100ms) should take care of this, but it does not always wait long enough.
    // So, add a wait time here:
    Wait.waitTime(200);

    // System.out.println(swName + " CanCoder " + getCanCoderDegrees() + " FX " + getTurningEncoderDegrees() + " pre-CAN");
    zeroDriveEncoder();
    // log.writeLogEcho(true, "SwerveModule", swName+" pre-CAN", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
    calibrateCanCoderDegrees(turningOffsetDegrees);
    // log.writeLogEcho(true, "SwerveModule", swName+" post-CAN", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
    calibrateTurningEncoderDegrees(getCanCoderDegrees());
    // log.writeLogEcho(true, "SwerveModule", swName+" post-FX", "Cancoder", getCanCoderDegrees(), "FX", getTurningEncoderDegrees());
  }

  /**
   * Sets the swerve module in coast or brake mode.
   * @param setCoast true = coast mode, false = brake mode
   */
  public void setMotorModeCoast(boolean setCoast) {
    if (setCoast) {
      driveMotor.setIdleMode(IdleMode.kCoast);
      turningMotor.setIdleMode(IdleMode.kCoast);
    } else {
      driveMotor.setIdleMode(IdleMode.kBrake);
      turningMotor.setIdleMode(IdleMode.kBrake);
    }
  }

  // ********** Main swerve module control methods

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getDriveEncoderVelocity(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Returns the current position of the module.
   *
   * @return The current position of the module.
   */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(
        getDriveEncoderMeters(), Rotation2d.fromDegrees(getTurningEncoderDegrees()));
  }

  /**
   * Turns off the drive and turning motors.
   */
  public void stopMotors() {
    driveMotor.set(0);
    turningMotor.set(0);
  }

  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setDriveMotorPercentOutput(double percentOutput){
    driveMotor.set(percentOutput);
  }
  
  /**
   * 
   * @param percentOutput Percent output to motor, -1 to +1
   */
  public void setTurnMotorPercentOutput(double percentOutput){
    turningMotor.set(percentOutput);
  }
  
  /**
   * Sets the desired state for the module, using closed loop controls on the Talons.
   * The Talons will hold this state until commanded to stop or another state.
   * @param desiredState Desired state with speed and angle
   * @param isOpenLoop true = fixed drive percent output to approximate velocity, false = closed loop drive velocity control
   */
  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    // Custom optimize command, since default WPILib optimize assumes continuous controller which CTRE is not
    desiredState =
        MathSwerveModuleState.optimize(desiredState, Rotation2d.fromDegrees(getTurningEncoderDegrees()));

    // Set drive motor velocity or percent output
    if(isOpenLoop){
      driveMotor.set(driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }
    else {
      drivePID.setReference(calculateDriveEncoderVelocityRaw(desiredState.speedMetersPerSecond), CANSparkMax.ControlType.kVelocity); // TODO check to see if it works
      // driveMotor.set(ControlMode.Velocity, calculateDriveEncoderVelocityRaw(desiredState.speedMetersPerSecond), 
      //   DemandType.ArbitraryFeedForward, driveFeedforward.calculate(desiredState.speedMetersPerSecond));
    }

    // Set turning motor target angle
    // TODO Determine the right way to implement this code.  Make it selectable by a boolean parameter to setDesiredState?
    // Prevent rotating module if speed is less then 1%. Prevents Jittering.
    // double angle = (Math.abs(desiredState.speedMetersPerSecond) <= (SwerveConstants.kMaxSpeedMetersPerSecond * 0.01)) 
    //   ? getTurningEncoderDegrees() : desiredState.angle.getDegrees(); 
    turningPID.setReference(calculateTurningEncoderTargetRaw(desiredState.angle.getDegrees()), CANSparkMax.ControlType.kPosition); // TODO check if first parameter makes sense
    // turningMotor.set(ControlMode.Position, calculateTurningEncoderTargetRaw(desiredState.angle.getDegrees())); 
  }

  // ********** Encoder methods

  // ******* Drive encoder methods

  /**
   * @return drive encoder position, in pinion rotations
   */
  public double getDriveEncoderRotations() {
    return driveEncoder.getPosition();
  }

  /**
	 * Set the drive encoder position to zero in software.
	 */
  public void zeroDriveEncoder() {
    driveEncoderZero = getDriveEncoderRotations();
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "ZeroDriveEncoder", "driveEncoderZero", driveEncoderZero, "raw encoder", getDriveEncoderRotations(), "encoder meters", getDriveEncoderMeters());
  }

  /**
   * @return drive wheel distance travelled, in meters (+ = forward)
   */
  public double getDriveEncoderMeters() {
    return (getDriveEncoderRotations() - driveEncoderZero) * SwerveConstants.kDriveEncoderMetersPerRotation;
  }

  /**
   * @return drive wheel velocity, in meters per second (+ = forward)
   */
  public double getDriveEncoderVelocity() {
    return driveEncoder.getVelocity() * SwerveConstants.kDriveEncoderMetersPerRotation / 60.0;
  }

  /**
   * Converts a target velocity (in meters per second) to a target raw drive motor velocity.
   * @param velocityMPS Desired drive wheel velocity, in meters per second (+ = forward)
   * @return drive motor raw value equivalent velocity
   */
  public double calculateDriveEncoderVelocityRaw(double velocityMPS) {
    return velocityMPS / SwerveConstants.kDriveEncoderMetersPerRotation * 60.0;
  }
  
  // ******* Turning motor encoder methods

  /**
   * @return turning motor encoder position, in rotations
   */
  public double getTurningEncoderRotations() {
    return turningEncoder.getPosition();
  }
  
  /**
   * Calibrates the turning motor encoder.  Sets the current wheel facing to currentAngleDegrees.
   * @param currentAngleDegrees current angle, in degrees.
   */
  public void calibrateTurningEncoderDegrees(double currentAngleDegrees) {
    turningEncoderZero = getTurningEncoderRotations() - (currentAngleDegrees / SwerveConstants.kTurningEncoderDegreesPerRotation);
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "calibrateTurningEncoder", "turningEncoderZero", turningEncoderZero, "raw encoder", getTurningEncoderRotations(), "set degrees", currentAngleDegrees, "encoder degrees", getTurningEncoderDegrees());
  }

  /**
   * @return turning motor encoder facing, in degrees.  Values do not wrap, so angle could be greater than 360 degrees.
   * When calibrated, 0 should be with the wheel pointing toward the front of robot.
   * + = counterclockwise, - = clockwise
   */
  public double getTurningEncoderDegrees() {
    return (getTurningEncoderRotations() - turningEncoderZero) * SwerveConstants.kTurningEncoderDegreesPerRotation;
  }

  /**
   * Converts a target wheel facing (in degrees) to a target raw turning motor encoder value.
   * @param targetDegrees Desired wheel facing, in degrees.  0 = towards front of robot, + = counterclockwise, - = clockwise
   * @return turning motor encoder raw value equivalent to input facing.
   */
  public double calculateTurningEncoderTargetRaw(double targetDegrees) {
    return targetDegrees / SwerveConstants.kTurningEncoderDegreesPerRotation + turningEncoderZero;
  }

  /**
   * @return turning motor encoder rotational velocity for wheel facing, in degrees per second.
   * + = counterclockwise, - = clockwise
   */
  public double getTurningEncoderVelocityDPS() {
    return turningEncoder.getVelocity() * SwerveConstants.kTurningEncoderDegreesPerRotation / 60.0;
  }

  // ******* Cancoder methods

  /**
   * Calibrates the CanCoder encoder, so that 0 should be with the wheel pointing toward the front of robot.
   * @param offsetDegrees Desired encoder zero point, in absolute magnet position reading
   */
  public void calibrateCanCoderDegrees(double offsetDegrees) {
    // System.out.println(swName + " " + turningOffsetDegrees);
    // turningCanCoder.configMagnetOffset(offsetDegrees, 100);
    cancoderZero = -offsetDegrees;
    log.writeLogEcho(true, buildString("SwerveModule ", swName), "calibrateCanCoder", 
      "cancoderZero", cancoderZero, 
      "raw encoder", turningCanCoderPosition.refresh().getValueAsDouble()*360.0, 
      "encoder degrees", getCanCoderDegrees());
  }

  /**
   * @return turning CanCoder wheel facing, in degrees [-180,+180).
   * When calibrated, 0 should be with the wheel pointing toward the front of robot.
   * + = counterclockwise, - = clockwise
   */
  public double getCanCoderDegrees() {
    return MathBCR.normalizeAngle(turningCanCoderPosition.refresh().getValueAsDouble()*360.0 - cancoderZero);
  }

  /**
   * @return turning CanCoder rotational velocity for wheel facing, in degrees per second.
   * + = counterclockwise, - = clockwise
   */
  public double getCanCoderVelocityDPS() {
    return turningCanCoderVelocity.refresh().getValueAsDouble()*360.0;
  }


  // ********** Information methods

  public double getDriveBusVoltage() {
    return driveMotor.getBusVoltage();
  }

  public double getDriveOutputPercent() {
    return driveMotor.getAppliedOutput();
  }

  public double getDriveStatorCurrent() {
    return driveMotor.getOutputCurrent();
  }

  public double getDriveTemp() {
    return driveMotor.getMotorTemperature();
  }

  public double getTurningOutputPercent() {
    return turningMotor.getAppliedOutput();
  }

  public double getTurningStatorCurrent() {
    return turningMotor.getOutputCurrent();
  }

  public double getTurningTemp() {
    return turningMotor.getMotorTemperature();
  }

  /**
   * Updates relevant variables on Shuffleboard
   */
  public void updateShuffleboard() {
    SmartDashboard.putNumber(buildString("Swerve FXangle ", swName), MathBCR.normalizeAngle(getTurningEncoderDegrees()));
    SmartDashboard.putNumber(buildString("Swerve CCangle ", swName), getCanCoderDegrees());
    SmartDashboard.putNumber(buildString("Swerve FXangle dps", swName), getTurningEncoderVelocityDPS());
    SmartDashboard.putNumber(buildString("Swerve distance", swName), getDriveEncoderMeters());
    SmartDashboard.putNumber(buildString("Swerve drive temp ", swName), getDriveTemp());
    SmartDashboard.putNumber(buildString("Swerve drive mps ", swName), getDriveEncoderVelocity());
  }

  /**
   * Returns information about the swerve module to include in the filelog
   * Format of the return string is comma-delimited name-value pairs, 
   * *without* the final comma.  Ex.  "name1,value1,name2,value2"
   */
  public String getLogString() {
    return buildString(
      swName, " CCangle deg,", getCanCoderDegrees(), ",",
      swName, " CCangle DPS,", getCanCoderVelocityDPS(), ",",
      swName, " FXangle deg,", MathBCR.normalizeAngle(getTurningEncoderDegrees()), ",",
      swName, " FXangle DPS,", getTurningEncoderVelocityDPS(), ",",
      swName, " turn output,", getTurningOutputPercent(), ",",
      swName, " drive meters,", getDriveEncoderMeters(), ",",
      swName, " drive mps,", getDriveEncoderVelocity(), ",",
      swName, " drive output,", getDriveOutputPercent(), ",",
      swName, " drive temp,", getDriveTemp(), ",",
      swName, " turn temp,", getTurningTemp()
    );
  }
}
