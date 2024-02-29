// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.*;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import frc.robot.utilities.StringUtil;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private final StatusSignal<Double> shooterTopCurrent;

  private final StatusSignal<Double> shooterBottomSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> shooterBottomTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> shooterBottomDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> shooterBottomStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> shooterBottomEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> shooterBottomEncoderVelocity;
  private final StatusSignal<Double> shooterBottomVoltage;
  private final StatusSignal<Double> shooterBottomCurrent;

  private final StatusSignal<Double> feederSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> feederTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> feederDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> feederStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> feederEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> feederEncoderVelocity;
  private final StatusSignal<Double> feederVoltage;
  private final StatusSignal<Double> feederCurrent;

  // Motor controls
  private VoltageOut motorVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage motorVelocityControl = new VelocityVoltage(0.0).withSlot(0);

  // Piece sensor inside the intake 
  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOIntakePieceSensor);

  // private SimpleMotorFeedforward motor1Feedforward = new SimpleMotorFeedforward(S, V, A); // TODO create and calibrate feed forward (or remove code)

  private boolean velocityControlOn = false;
  private double setpointRPM1;
  private double setpointRPM2;
  private double measuredRPM1 = 0.0;
  private double measuredRPM2 = 0.0;
  
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
    shooterTopCurrent = shooterTop.getSupplyCurrent();

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
    shooterBottomCurrent = shooterBottom.getSupplyCurrent();

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
    feederCurrent = feeder.getSupplyCurrent();

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
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
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

  /**
   * sets the percent of the top and bottom shooter motors, and the feeder using voltage compensation
   * @param shooterPercent percent for both shooter motors, -1.0 to +1.0 (+ = shoot forward)
   * @param feederPercent percent for the feeder motor, 
   */
  public void setMotorPercentOutput(double shooterPercent, double feederPercent) {
    setMotorPercentOutput(shooterPercent, shooterPercent, feederPercent);
  }

  /**
   * sets the percent of motor 1, 2, and feeder using voltage compensation if turned on
   * @param shooter1Percent percent for the shooter motor 1
   * @param shooter2Percent percent for the shooter motor 2
   * @param feederPercent percent for the feeder
   */
  public void setMotorPercentOutput(double shooter1Percent, double shooter2Percent, double feederPercent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    shooterTop.setControl(motorVoltageControl.withOutput(shooter1Percent * ShooterConstants.compensationVoltage));
    shooterBottom.setControl(motorVoltageControl.withOutput(shooter2Percent * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM1 = 0.0;
    setpointRPM2 = 0.0;
    feeder.setControl(motorVoltageControl.withOutput(feederPercent * ShooterConstants.compensationVoltage));
  }
  
  /**
   * sets the percent of motor 1 and 2 using voltage compensation if turned on
   * @param percent percent for the shooter
   */
  public void setShooterMotorPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    shooterTop.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
    shooterBottom.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM1 = 0.0;
    setpointRPM2 = 0.0;
  }

  /**
   * sets the percent of motor 1 and 2 using voltage compensation if turned on
   * @param percent1 percent for the shooter motor 1
   * @param percent2 percent for the shooter motor 2
   */
  public void setShooterMotorPercentOutput(double percent1, double percent2) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    shooterTop.setControl(motorVoltageControl.withOutput(percent1 * ShooterConstants.compensationVoltage));
    shooterBottom.setControl(motorVoltageControl.withOutput(percent2 * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM1 = 0.0;
    setpointRPM2 = 0.0;
  }

  /**
   * sets the percent of the feeder, using voltage compensation if turned on
   * @param percent percent for the feeder
   */
  public void setFeederMotorPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    feeder.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
  * Stops motor 1, 2, and feeder
  */
  public void stopMotors() {
    setMotorPercentOutput(0.0, 0.0);
    velocityControlOn = false;
    setpointRPM1 = 0.0;
    setpointRPM2 = 0.0;
  }

  /**
   * Returns the shooter motor 1 position
   * @return position of shooter motor in raw units, without software zeroing
   */
  public double getShooter1PositionRaw() {
    shooterTopEncoderPosition.refresh();
    return shooterTopEncoderPosition.getValueAsDouble();
  }

  /**
   * Returns the shooter motor 2 position
   * @return position of shooter motor in raw units, without software zeroing
   */
  public double getShooter2PositionRaw() {
    shooterBottomEncoderPosition.refresh();
    return shooterBottomEncoderPosition.getValueAsDouble();
  }

  /**
   * Returns the feeder motor position
   * @return position of feeder motor in raw units, without software zeroing
   */
  public double getFeederPositionRaw() {
    feederEncoderPosition.refresh();
    return feederEncoderPosition.getValueAsDouble();
  }

  // /**
  //  * Returns the shooter motor 1 position
  //  * @return position of shooter motor in revolutions
  //  */
  // public double getShooterPosition() {
  //   return (getShooterPositionRaw() - shooterEncoderZero) / ShooterConstants.ticksPerRevolution;
  // }

  // /**
  //  * Returns the feeder motor position
  //  * @return position of shooter motor in revolutions
  //  */
  // public double getFeederPosition() {
  //   return (getFeederPositionRaw() - feederEncoderZero) / ShooterConstants.ticksPerRevolution;
  // }

  // /**
	//  * Zero the encoder position in software.
	//  */
  // public void zeroEncoder() {
  //   shooterEncoderZero = getShooterPositionRaw();
  //   feederEncoderZero = getFeederPositionRaw();
  // }

  /**
   * @return velocity of shooter motor 1 in rpm
   */
  public double getShooter1Velocity() {
    shooterTopEncoderVelocity.refresh();
    return shooterTopEncoderVelocity.getValueAsDouble()*60;
  }

  /**
   * @return velocity of shooter motor 2 in rpm
   */
  public double getShooter2Velocity() {
    shooterBottomEncoderVelocity.refresh();
    return shooterBottomEncoderVelocity.getValueAsDouble()*60;
  }

  /**
   * @return velocity of feeder in rpm
   */
  public double getFeederVelocity() {
    feederEncoderVelocity.refresh();
    return feederEncoderVelocity.getValueAsDouble()*60;
  }

  /**
   * @param rpm velocity of shooter motors in rpm
   */
  public void setShooterVelocity(double rpm) { 
    velocityControlOn = true;
    setpointRPM1 = rpm;
    setpointRPM2 = rpm;
    shooterTop.setControl(motorVelocityControl.withVelocity(rpm/60));
    shooterBottom.setControl(motorVelocityControl.withVelocity(rpm/60));
  }

  /**
   * @param rpm1 velocity of shooter motor 1 in rpm
   * @param rpm2 velocity of shooter motor 2 in rpm
   */
  public void setShooterVelocity(double rpm1, double rpm2) { 
    velocityControlOn = true;
    setpointRPM1 = rpm1;
    setpointRPM2 = rpm2;
    shooterTop.setControl(motorVelocityControl.withVelocity(rpm1/60));
    shooterBottom.setControl(motorVelocityControl.withVelocity(rpm2/60));
  }

  /**
   * @return difference between measured RPM and set point RPM for shooter motor 1
   */
  public double getVelocity1PIDError() {
    return measuredRPM1 - setpointRPM1;
  }

  /**
   * @return difference between measured RPM and set point RPM for shooter motor 2
   */
  public double getVelocity2PIDError() {
    return measuredRPM2 - setpointRPM2;
  }

  /**
   * Reads the top shooter voltage
   * @return top shooter voltage, in volts
   */
  public double getShooterTopVoltage() {
    return shooterTopVoltage.refresh().getValueAsDouble();
  }

  /**
   * Reads the bottom shooter voltage
   * @return bottom shooter voltage, in volts
   */
  public double getShooterBottomVoltage() {
    return shooterBottomVoltage.refresh().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // Update the measured RPM
    measuredRPM1 = getShooter1Velocity();
    measuredRPM2 = getShooter2Velocity();

    // Log
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Voltage"), shooterTopVoltage.refresh().getValueAsDouble());
     // SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Position Rev"), getShooterPosition());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Velocity 1 RPM"), measuredRPM1);
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Velocity 2 RPM"), measuredRPM2);
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Temperature C"), shooterTopTemp.refresh().getValueAsDouble());
    }
  }

  /**
   * Update information about shooter to fileLog
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
  public void updateLog(boolean logWhenDisabled) {
    // Log shooter
    log.writeLog(
      logWhenDisabled,
      subsystemName,
      "Update Variables",
      "Bus Volt", shooterTopSupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent 1", shooterTopDutyCycle.refresh().getValueAsDouble(),
      "Out Percent 2", shooterBottomDutyCycle.refresh().getValueAsDouble(),
      "Volt 1", shooterTopVoltage.refresh().getValueAsDouble(),
      "Volt 2", shooterBottomVoltage.refresh().getValueAsDouble(),
      "Amps 1", shooterTopCurrent.refresh().getValueAsDouble(),
      "Amps 2", shooterBottomCurrent.refresh().getValueAsDouble(),
      "Temperature 1", shooterTopTemp.refresh().getValueAsDouble(),
      "Temperature 2", shooterBottomTemp.refresh().getValueAsDouble(),
    //  "Position", getShooterPosition(),
      "Measured RPM 1", measuredRPM1,
      "Measured RPM 2", measuredRPM2,
      "Setpoint RPM 1", setpointRPM1,
      "Setpoint RPM 2", setpointRPM2
    );
    // Log feeder
    log.writeLog(
      logWhenDisabled,
      "Feeder",
      "Update Variables",
      "Bus Volt", feederSupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent 1", feederDutyCycle.refresh().getValueAsDouble(),
      "Volt 1", feederVoltage.refresh().getValueAsDouble(),
      "Amps 1", feederCurrent.refresh().getValueAsDouble(),
      "Temperature 1", feederTemp.refresh().getValueAsDouble()
     // "Position", getFeederPosition()
    );
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }
}
