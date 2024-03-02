// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.StringUtil;


public class Shooter extends SubsystemBase implements Loggable {
  private final FileLog log;
  private boolean fastLogging = false;
  private int logRotationKey;
  private final String subsystemName;

  // Create Kraken for top shooter motor
  private final TalonFX shooterTop = new TalonFX(Ports.CANShooterTop);
	private final TalonFXConfigurator shooterTopConfigurator;
	private TalonFXConfiguration shooterTopConfig;

  // Create Kraken for bottom shooter motor
  private final TalonFX shooterBottom = new TalonFX(Ports.CANShooterBottom);
	private final TalonFXConfigurator shooterBottomConfigurator;
  private TalonFXConfiguration shooterBottomConfig;

  // Create Kraken for feeder motor
  private final TalonFX feeder = new TalonFX(Ports.CANFeeder);
  private final TalonFXConfigurator feederConfigurator;
  private TalonFXConfiguration feederConfig;

  // Create signals and sensors for motors
  private final StatusSignal<Double> shooterTopSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> shooterTopTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> shooterTopDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> shooterTopStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> shooterTopEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> shooterTopEncoderVelocity;
  private final StatusSignal<Double> shooterTopVoltage;

  private final StatusSignal<Double> shooterBottomSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> shooterBottomTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> shooterBottomDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> shooterBottomStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> shooterBottomEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> shooterBottomEncoderVelocity;
  private final StatusSignal<Double> shooterBottomVoltage;

  private final StatusSignal<Double> feederSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> feederTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> feederDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> feederStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> feederEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> feederEncoderVelocity;
  private final StatusSignal<Double> feederVoltage;

  // Motor controls
  private VoltageOut motorVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage motorVelocityControl = new VelocityVoltage(0.0).withSlot(0);

  // Piece sensor inside the intake 
  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOFeederPieceSensor);

  // private SimpleMotorFeedforward motor1Feedforward = new SimpleMotorFeedforward(S, V, A); // TODO create and calibrate feed forward (or remove code)

  private boolean velocityControlOn = false;
  private double setpointRPMTop;
  private double setpointRPMBottom;
  
  /**
   * Create the shooter subsystem
   * @param log
   */
  public Shooter(FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();
    subsystemName = "Shooter";

    // Configure top shooter motor
    shooterTopConfigurator = shooterTop.getConfigurator();
    shooterTopSupplyVoltage = shooterTop.getSupplyVoltage();
	  shooterTopTemp = shooterTop.getDeviceTemp();
	  shooterTopDutyCycle = shooterTop.getDutyCycle();
	  shooterTopStatorCurrent = shooterTop.getStatorCurrent();
	  shooterTopEncoderPosition = shooterTop.getPosition();
	  shooterTopEncoderVelocity = shooterTop.getVelocity();
    shooterTopVoltage = shooterTop.getMotorVoltage();

    shooterTopConfig = new TalonFXConfiguration();			// Factory default configuration
    shooterTopConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Do not invert motor
		shooterTopConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;      // Coast mode to reduce wear on motor
    shooterTopConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;         // # seconds from 0 to full power
		shooterTopConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;     // # seconds from 0 to full power
    
    // Configure bottom shooter motor
    shooterBottomConfigurator = shooterBottom.getConfigurator();
    shooterBottomSupplyVoltage = shooterBottom.getSupplyVoltage();
	  shooterBottomTemp = shooterBottom.getDeviceTemp();
	  shooterBottomDutyCycle = shooterBottom.getDutyCycle();
	  shooterBottomStatorCurrent =shooterBottom.getStatorCurrent();
	  shooterBottomEncoderPosition = shooterBottom.getPosition();
	  shooterBottomEncoderVelocity = shooterBottom.getVelocity();
    shooterBottomVoltage = shooterBottom.getMotorVoltage();

    shooterBottomConfig = new TalonFXConfiguration();			// Factory default configuration
    shooterBottomConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		shooterBottomConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;     // Coast mode to reduce wear on motor
    shooterBottomConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;        // # seconds from 0 to full power
		shooterBottomConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;    // # seconds from 0 to full power

    // Configure feeder
    feederConfigurator = feeder.getConfigurator();
    feederSupplyVoltage = feeder.getSupplyVoltage();
	  feederTemp = feeder.getDeviceTemp();
	  feederDutyCycle = feeder.getDutyCycle();
	  feederStatorCurrent =feeder.getStatorCurrent();
	  feederEncoderPosition = feeder.getPosition();
	  feederEncoderVelocity = feeder.getVelocity();
    feederVoltage = feeder.getMotorVoltage();

    feederConfig = new TalonFXConfiguration();			// Factory default configuration
    feederConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;      // Brake mode to hold piece in feeder
    feederConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.1;         // # seconds from 0 to full power
		feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.1;     // # seconds from 0 to full power
    
    // Make motor2 follow motor1
    //motor2.setControl(new Follower(motor1.getDeviceID(), false)); 

    // Set PID coefficients
    motorVelocityControl.Slot = 0;
    motorVelocityControl.OverrideBrakeDurNeutral = false;
    shooterTopConfig.Slot0.kP = ShooterConstants.ShooterTopkP;
    shooterTopConfig.Slot0.kI = ShooterConstants.ShooterTopkI;
    shooterTopConfig.Slot0.kD = ShooterConstants.ShooterTopkD;
    shooterTopConfig.Slot0.kS = ShooterConstants.ShooterTopkS;
    shooterTopConfig.Slot0.kV = ShooterConstants.ShooterTopkV;
    shooterTopConfig.Slot0.kA = ShooterConstants.ShooterTopkA;

    shooterBottomConfig.Slot0.kP = ShooterConstants.ShooterBottomkP;
    shooterBottomConfig.Slot0.kI = ShooterConstants.ShooterBottomkI;
    shooterBottomConfig.Slot0.kD = ShooterConstants.ShooterBottomkD;
    shooterBottomConfig.Slot0.kS = ShooterConstants.ShooterBottomkS;
    shooterBottomConfig.Slot0.kV = ShooterConstants.ShooterBottomkV;
    shooterBottomConfig.Slot0.kA = ShooterConstants.ShooterBottomkA;

    feederConfig.Slot0.kP = ShooterConstants.FeederkP;
    feederConfig.Slot0.kI = ShooterConstants.FeederkI;
    feederConfig.Slot0.kD = ShooterConstants.FeederkD;
    feederConfig.Slot0.kS = ShooterConstants.FeederkS;
    feederConfig.Slot0.kV = ShooterConstants.FeederkV;
    feederConfig.Slot0.kA = ShooterConstants.FeederkA;

    // Apply configuration to all the motors.  
		// This is a blocking call and will wait up to 50ms-70ms for each config to apply.  (initial test = 62ms delay)
    shooterTopConfigurator.apply(shooterTopConfig);
    shooterBottomConfigurator.apply(shooterBottomConfig);
    feederConfigurator.apply(feederConfig);

    // Stop all motors
    stopMotors();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  // *** Percent motor controls

  /**
   * Sets the percent of the top and bottom shooter motors, and the feeder.  All motors use voltage compensation.
   * @param shooterPercent percent for both shooter motors, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   * @param feederPercent percent for the feeder motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   */
  public void setMotorsPercentOutput(double shooterPercent, double feederPercent) {
    setShooterPercentOutput(shooterPercent, shooterPercent);
    setFeederPercentOutput(feederPercent);
  }

  /**
   * Sets the percent of the top and bottom shooter motors, and the feeder.  All motors use voltage compensation.
   * @param shooterTopPercent percent for the top shooter motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   * @param shooterBottomPercent percent for the bottom shooter motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   * @param feederPercent percent for the feeder, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   */
  public void setMotorsPercentOutput(double shooterTopPercent, double shooterBottomPercent, double feederPercent) {
    setShooterPercentOutput(shooterTopPercent, shooterBottomPercent);
    setFeederPercentOutput(feederPercent);
  }
  
  /**
   * Sets the percent of the top and bottom shooter motors.  All motors use voltage compensation.
   * @param percent percent for the shooter motors, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   */
  public void setShooterPercentOutput(double percent) {
    setShooterPercentOutput(percent, percent);
  }

  /**
   * Sets the percent of the top and bottom shooter motors.  All motors use voltage compensation.
   * @param shooterTopPercent percent for the top shooter motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   * @param shooterBottomPercent percent for the bottom shooter motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   */
  public void setShooterPercentOutput(double shooterTopPercent, double shooterBottomPercent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    shooterTop.setControl(motorVoltageControl.withOutput(shooterTopPercent * ShooterConstants.compensationVoltage));
    shooterBottom.setControl(motorVoltageControl.withOutput(shooterBottomPercent * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPMTop = 0.0;
    setpointRPMBottom = 0.0;
  }

  /**
   * Sets the percent of the feeder motor, using voltage compensation.
   * @param percent percent for the feeder motor, -1.0 to +1.0 (+ = shoot forward, - = backwards)
   */
  public void setFeederPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    feeder.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
  * Stops both shooter motors and the feeder motor
  */
  public void stopMotors() {
    setShooterPercentOutput(0.0, 0.0);
    setFeederPercentOutput(0.0);
  }

  // *** Velocity motor controls

  /**
   * Sets the target velocity for both shooter wheels
   * @param rpm velocity of shooter wheels in rpm
   */
  public void setShooterVelocity(double rpm) { 
    setShooterVelocity(rpm, rpm);
  }

  /**
   * Sets the target velocity for both shooter wheels
   * @param rpmTop velocity of top shooter wheel in rpm
   * @param rpmBottom velocity of bottom shooter wheel in rpm
   */
  public void setShooterVelocity(double rpmTop, double rpmBottom) { 
    velocityControlOn = true;
    setpointRPMTop = rpmTop;
    setpointRPMBottom = rpmBottom;
    shooterTop.setControl(motorVelocityControl.withVelocity(rpmTop/60.0/ShooterConstants.shooterGearRatio));
    shooterBottom.setControl(motorVelocityControl.withVelocity(rpmBottom/60.0/ShooterConstants.shooterGearRatio));
  }

  /**
   * @return difference between measured RPM and set point RPM for top shooter wheel
   */
  public double getTopShooterVelocityPIDError() {
    return getTopShooterVelocity() - setpointRPMTop;
  }

  /**
   * @return difference between measured RPM and set point RPM for bottom shooter wheel
   */
  public double getBottomShooterVelocityPIDError() {
    return getBottomShooterVelocity() - setpointRPMBottom;
  }

  // *** Motor sensors

  /**
   * Returns the top shooter position
   * @return position of shooter in wheel rotations
   */
  public double getTopShooterPosition() {
    shooterTopEncoderPosition.refresh();
    return shooterTopEncoderPosition.getValueAsDouble() * ShooterConstants.shooterGearRatio;
  }

  /**
   * Returns the bottom shooter position
   * @return position of shooter in wheel rotations
   */
  public double getBottomShooterPosition() {
    shooterBottomEncoderPosition.refresh();
    return shooterBottomEncoderPosition.getValueAsDouble() * ShooterConstants.shooterGearRatio;
  }

  /**
   * Returns the feeder position
   * @return position of feeder in wheel rotations
   */
  public double getFeederPosition() {
    feederEncoderPosition.refresh();
    return feederEncoderPosition.getValueAsDouble() * ShooterConstants.feederGearRatio;
  }

  /**
   * @return velocity of top shooter in wheel rpm
   */
  public double getTopShooterVelocity() {
    shooterTopEncoderVelocity.refresh();
    return shooterTopEncoderVelocity.getValueAsDouble() * 60.0 * ShooterConstants.shooterGearRatio;
  }

  /**
   * @return velocity of bottom shooter in wheel rpm
   */
  public double getBottomShooterVelocity() {
    shooterBottomEncoderVelocity.refresh();
    return shooterBottomEncoderVelocity.getValueAsDouble() * 60.0 * ShooterConstants.shooterGearRatio;
  }

  /**
   * @return velocity of feeder in wheel rpm
   */
  public double getFeederVelocity() {
    feederEncoderVelocity.refresh();
    return feederEncoderVelocity.getValueAsDouble() * 60.0 * ShooterConstants.feederGearRatio;
  }

  /**
   * Reads the top shooter voltage applied to the motor
   * @return top shooter voltage, in volts
   */
  public double getTopShooterVoltage() {
    return shooterTopVoltage.refresh().getValueAsDouble();
  }

  /**
   * Reads the bottom shooter voltage applied to the motor
   * @return bottom shooter voltage, in volts
   */
  public double getBottomShooterVoltage() {
    return shooterBottomVoltage.refresh().getValueAsDouble();
  }

  // ***  Piece sensor

  /**
   * 
   * @return true if piece is in feeder
   */
  public boolean isPiecePresent(){
    return !pieceSensor.get();
  }


  @Override
  public void periodic() {
    // Log
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);

      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Voltage"), getTopShooterVoltage());
     // SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Position Rev"), getShooterPosition());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Top RPM"), getTopShooterVelocity());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Bottom RPM"), getBottomShooterVelocity());
      SmartDashboard.putNumber(StringUtil.buildString("Feeder RPM"), getFeederVelocity());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Top Temp C"), shooterTopTemp.refresh().getValueAsDouble());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Bottom Temp C"), shooterBottomTemp.refresh().getValueAsDouble());
      SmartDashboard.putNumber(StringUtil.buildString("Feeder Temp C"), feederTemp.refresh().getValueAsDouble());
      SmartDashboard.putBoolean("Feeder has piece", isPiecePresent());
    }
  }

  /**
   * Update information about shooter to fileLog
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    log.writeLog(
      logWhenDisabled,
      subsystemName,
      "Update Variables",
      "Bus Volt", shooterTopSupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent Top", shooterTopDutyCycle.refresh().getValueAsDouble(),
      "Out Percent Bottom", shooterBottomDutyCycle.refresh().getValueAsDouble(),
      "Out Percent Feed", feederDutyCycle.refresh().getValueAsDouble(),
      "Volt Top", getTopShooterVoltage(),
      "Volt Bottom", getBottomShooterVoltage(),
      "Volt Feed", feederVoltage.refresh().getValueAsDouble(),
      "Amps Top", shooterTopStatorCurrent.refresh().getValueAsDouble(),
      "Amps Bottom", shooterBottomStatorCurrent.refresh().getValueAsDouble(),
      "Amps Feed", feederStatorCurrent.refresh().getValueAsDouble(),
      "Temp Top", shooterTopTemp.refresh().getValueAsDouble(),
      "Temp Bottom", shooterBottomTemp.refresh().getValueAsDouble(),
      "Temp Feed", feederTemp.refresh().getValueAsDouble(),
      "Meas RPM Top", getTopShooterVelocity(),
      "Meas RPM Bottom", getBottomShooterVelocity(),
      "Meas RPM Feed", getFeederVelocity(),
      "Velocity Control", velocityControlOn,
      "Set RPM Top", setpointRPMTop,
      "Set RPM Bottom", setpointRPMBottom,
      "Piece in Feeder", isPiecePresent()
    );
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }
}
