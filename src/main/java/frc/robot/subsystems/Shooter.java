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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase implements Loggable {
  private final FileLog log;

  private final String subsystemName;
  private final TalonFX motor1;
	private final TalonFXConfigurator motor1Configurator;
	private TalonFXConfiguration motor1Config;
  private final TalonFX motor2;
	private final TalonFXConfigurator motor2Configurator;
  private TalonFXConfiguration motor2Config;
  private final TalonFX feeder;
  private final TalonFXConfigurator feederConfigurator;
  private TalonFXConfiguration feederConfig;
  private final StatusSignal<Double> motor1SupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> motor1Temp;				// Motor temperature, in degC
	private final StatusSignal<Double> motor1DutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> motor1StatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> motor1EncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> motor1EncoderVelocity;
  private final StatusSignal<Double> motor1Voltage;
  private final StatusSignal<Double> motor1Current;
  private final StatusSignal<Double> motor2SupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> motor2Temp;				// Motor temperature, in degC
	private final StatusSignal<Double> motor2DutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> motor2StatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> motor2EncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> motor2EncoderVelocity;
  private final StatusSignal<Double> motor2Voltage;
  private final StatusSignal<Double> motor2Current;
  private final StatusSignal<Double> feederSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> feederTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> feederDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> feederStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> feederEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> feederEncoderVelocity;
  private final StatusSignal<Double> feederVoltage;
  private final StatusSignal<Double> feederCurrent;


  private SimpleMotorFeedforward motor1Feedforward; // todo

  private VoltageOut motorVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage motorVelocityControl = new VelocityVoltage(0.0).withSlot(0);

  private boolean velocityControlOn;
  private double setpointRPM;
  private double shooterEncoderZero = 0.0;
  private double feederEncoderZero = 0.0;
  private double measuredRPM = 0.0;
  private boolean fastLogging = true;
  private int logRotationKey;
  
  /** Creates a new Shooter. */
  public Shooter(FileLog log) {
    this.log = log;
    logRotationKey = log.allocateLogRotation();

    motor1 = new TalonFX(Ports.CANShooter1);
    motor2 = new TalonFX(Ports.CANShooter2);
    feeder = new TalonFX(Ports.CANFeeder);
    subsystemName = "Shooter";
    // Configure motor2
    motor1Configurator = motor1.getConfigurator();
    motor1SupplyVoltage = motor1.getSupplyVoltage();
	  motor1Temp = motor1.getDeviceTemp();
	  motor1DutyCycle = motor1.getDutyCycle();
	  motor1StatorCurrent = motor1.getStatorCurrent();
	  motor1EncoderPosition = motor1.getPosition();
	  motor1EncoderVelocity = motor1.getVelocity();
    motor1Voltage = motor1.getMotorVoltage();
    motor1Current = motor1.getSupplyCurrent();

    motor1Config = new TalonFXConfiguration();			// Factory default configuration
    motor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    motor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		motor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    
    // Configure motor2
    motor2Configurator = motor2.getConfigurator();
    motor2SupplyVoltage = motor2.getSupplyVoltage();
	  motor2Temp = motor2.getDeviceTemp();
	  motor2DutyCycle = motor2.getDutyCycle();
	  motor2StatorCurrent =motor2.getStatorCurrent();
	  motor2EncoderPosition = motor2.getPosition();
	  motor2EncoderVelocity = motor2.getVelocity();
    motor2Voltage = motor2.getMotorVoltage();
    motor2Current = motor2.getSupplyCurrent();

    motor2Config = new TalonFXConfiguration();			// Factory default configuration
    motor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		motor2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    motor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		motor2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;

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
		feederConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    feederConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		feederConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    
    // Make motor2 follow motor1
    motor2.setControl(new Follower(motor1.getDeviceID(), false)); 

    // Set the PID and stop the motor 1
    setPIDSVA(
      ShooterConstants.Motor1kP,
      ShooterConstants.Motor1kI,
      ShooterConstants.Motor1kD,
      ShooterConstants.Motor1kS,
      ShooterConstants.Motor1kV,
      ShooterConstants.Motor1kA,
      ShooterConstants.Motor2kP,
      ShooterConstants.Motor2kP,
      ShooterConstants.Motor2kI,
      ShooterConstants.Motor2kD,
      ShooterConstants.Motor2kS,
      ShooterConstants.Motor2kV,
      ShooterConstants.Motor2kA,
      ShooterConstants.FeederkP,
      ShooterConstants.FeederkI,
      ShooterConstants.FeederkD,
      ShooterConstants.FeederkS,
      ShooterConstants.FeederkV
    );
    stopMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * Sets the voltage of the motor
   * 
   * <p> Compensates for the current bus
	 * voltage to ensure that the desired voltage is output even if the battery voltage is below
	 * 12V - highly useful when the voltage outputs are "meaningful" (e.g. they come from a
	 * feedforward calculation).
	 *
	 * <p>NOTE: This function *must* be called regularly in order for voltage compensation to work
	 * properly - unlike the ordinary set function, it is not "set it and forget it."
	 *
   * @param voltage voltage
   */
  public void setVoltage(double voltage) {
    motor1.setVoltage(voltage);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param shooterPercent percent for the shooter
   * @param feederPercent percent for the feeder
   */
  public void setMotorPercentOutput(double shooterPercent, double feederPercent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    motor1.setControl(motorVoltageControl.withOutput(shooterPercent * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM = 0.0;
    feeder.setControl(motorVoltageControl.withOutput(feederPercent * ShooterConstants.compensationVoltage));
  }
  
  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param percent percent for the shooter
   */
  public void setShooterMotorPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    motor1.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param percent percent for the feeder
   */
  public void setFeederMotorPercentOutput(double percent) {
    // Percent output control does not exist; multiply compensationVoltage by percent
    feeder.setControl(motorVoltageControl.withOutput(percent * ShooterConstants.compensationVoltage));
  }

  /**
  * Stops the motor
  */
  public void stopMotor() {
    setMotorPercentOutput(0.0, 0.0);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
   * Returns the shooter motor 1 position
   * @return position of shooter motor in raw units, without software zeroing
   */
  public double getShooterPositionRaw() {
    motor1EncoderPosition.refresh();
    return motor1EncoderPosition.getValueAsDouble();
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
  public double getShooterVelocity() {
    motor1EncoderVelocity.refresh();
    return motor1EncoderVelocity.getValueAsDouble();
  }

  /**
   * @return velocity of feeder motor 1 in rpm
   */
  public double getFeederVelocity() {
    feederEncoderVelocity.refresh();
    return feederEncoderVelocity.getValueAsDouble();
  }

  /**
   * @param velocity of shooter motor 1 in rpm
   */
  public void setShooterVelocity(double rpm) { 
    velocityControlOn = true;
    setpointRPM = rpm;
    motor1.setControl(motorVelocityControl.withVelocity(rpm/60));
  }

  /**
   * @param P
   * @param I
   * @param D
   * @param S
   * @param V
   * @param A
   */
  public void setPIDSVA(double Motor1P, double Motor1I, double Motor1D, double Motor1S, double Motor1V, double Motor1A, double Motor2P, double Motor2I, double Motor2D, double Motor2S, double Motor2V, double Motor2A, double FeederP, double FeederI, double FeederD, double FeederS, double FeederV, double FeederA) {
    // Set PID coefficients
    motorVelocityControl.Slot = 0;
    motorVelocityControl.OverrideBrakeDurNeutral = true;
    motor1Config.Slot0.kP = Motor1P;
    motor1Config.Slot0.kI = Motor1I;
    motor1Config.Slot0.kD = Motor1D;
    motor1Config.Slot0.kV = Motor1V;
    motor1Config.Slot0.kS = Motor1S;

    motor2Config.Slot0.kP = Motor2P;
    motor2Config.Slot0.kI = Motor2I;
    motor2Config.Slot0.kD = Motor2D;
    motor2Config.Slot0.kV = Motor2V;
    motor2Config.Slot0.kS = Motor2S;



    feederConfig.Slot0.kP = FeederP;
    feederConfig.Slot0.kP = FeederP;
    feederConfig.Slot0.kP = FeederP;
    feederConfig.Slot0.kP = FeederP;
    feederConfig.Slot0.kP = FeederP;

    // Apply configuration to the motor1.  
		// This is a blocking call and will wait up to 50ms-70ms for the config to apply.  (initial test = 62ms delay)
    motor1Configurator.apply(motor1Config);

    motor2Configurator.apply(motor2Config);

    feederConfigurator.apply(feederConfig);
    // motor1Feedforward = new SimpleMotorFeedforward(S, V, A);
    if (velocityControlOn) {
      // Reset velocity to force kS and kV updates to take effect
      setShooterVelocity(setpointRPM);
    }
  
  }

  /**
   * @return difference between measured RPM and set point RPM
   */
  public double getVelocityPIDError() {
    return measuredRPM - setpointRPM;
  }

  @Override
  public void periodic() {
    // Update the measured RPM
    measuredRPM = getShooterVelocity();

    // Log
    if (fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Voltage"), motor1Voltage.refresh().getValueAsDouble());
     // SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Position Rev"), getShooterPosition());
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Velocity RPM"), measuredRPM);
      SmartDashboard.putNumber(StringUtil.buildString(subsystemName, " Temperature C"), motor1Temp.refresh().getValueAsDouble());
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
      "Bus Volt", motor1SupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent 1", motor1DutyCycle.refresh().getValueAsDouble(),
      "Out Percent 2", motor2DutyCycle.refresh().getValueAsDouble(),
      "Volt 1", motor1Voltage.refresh().getValueAsDouble(),
      "Volt 2", motor2Voltage.refresh().getValueAsDouble(),
      "Amps 1", motor1Current.refresh().getValueAsDouble(),
      "Amps 2", motor2Current.refresh().getValueAsDouble(),
      "Temperature 1", motor1Temp.refresh().getValueAsDouble(),
      "Temperature 2", motor2Temp.refresh().getValueAsDouble(),
    //  "Position", getShooterPosition(),
      "Measured RPM", measuredRPM,
      "Setpoint RPM", setpointRPM
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
