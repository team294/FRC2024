// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants.Ports;
import frc.robot.Constants.SwerveConstants;
import frc.robot.utilities.FileLog;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  /** Creates a new Shooter. */
  private final String subsystemName;
  private final TalonFX motor1;
	private final TalonFXConfigurator motor1Configurator;
	private TalonFXConfiguration motor1Config;
  private final TalonFX motor2;
	private final TalonFXConfigurator motor2Configurator;
	private TalonFXConfiguration motor2Config;
  private final StatusSignal<Double> motor1SupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> motor1Temp;				// Motor temperature, in degC
	private final StatusSignal<Double> motor1DutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> motor1StatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> motor1EncoderPostion;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> motor1EncoderVelocity;	
  private final StatusSignal<Double> motor2SupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> motor2Temp;				// Motor temperature, in degC
	private final StatusSignal<Double> motor2DutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> motor2StatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> motor2EncoderPostion;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> motor2EncoderVelocity;	
  public Shooter(String subsytemName, FileLog log) {
    motor1 = new TalonFX(Ports.CANShooter1);
    motor2 = new TalonFX(Ports.CANShooter2);
    this.subsystemName = subsytemName;
    // Configure motor2
    motor1Configurator = motor1.getConfigurator();
    motor1SupplyVoltage = motor1.getSupplyVoltage();
	  motor1Temp = motor1.getDeviceTemp();
	  motor1DutyCycle = motor1.getDutyCycle();
	  motor1StatorCurrent = motor1.getStatorCurrent();
	  motor1EncoderPostion = motor1.getPosition();
	  motor1EncoderVelocity = motor1.getVelocity();

    motor1Config = new TalonFXConfiguration();			// Factory default configuration
    motor1Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		motor1Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    motor1Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		motor1Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    
    // Configure motor2
    motor2Configurator = motor1.getConfigurator();
    motor2SupplyVoltage = motor1.getSupplyVoltage();
	  motor2Temp = motor1.getDeviceTemp();
	  motor2DutyCycle = motor1.getDutyCycle();
	  motor2StatorCurrent =motor1.getStatorCurrent();
	  motor2EncoderPostion = motor1.getPosition();
	  motor2EncoderVelocity = motor1.getVelocity();

    motor2Config = new TalonFXConfiguration();			// Factory default configuration
    motor2Config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		motor2Config.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    motor2Config.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		motor2Config.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    

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
   * @param percent percent
   */
  public void setMotorPercentOutput(double percent){
    //motor.set(ControlMode.PercentOutput, percent);
    motor.setControl(motorVoltageControl.withOutput(percent*FalconConstants.compensationVoltage));
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  /**
  * Stops the motor
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }


  /**
   * Returns the motor position
   * @return position of motor in raw units, without software zeroing
   */
  public double getMotorPositionRaw(){
    //return motor.getSelectedSensorPosition(0);
    motorEncoderPosition.refresh();
    return motorEncoderPosition.getValueAsDouble();
  }

  /**
   * Returns the motor position
   * @return position of motor in revolutions
   */
  public double getMotorPosition(){
    return (getMotorPositionRaw() - encoderZero)/FalconConstants.ticksPerRevolution;
  }

  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  /**
   * @return velocity of motor in rpm
   */
  public double getMotorVelocity(){
    motorEncoderVelocity.refresh();
    return motorEncoderVelocity.getValueAsDouble();
    // return motor.getSelectedSensorVelocity(0)*FalconConstants.rawVelocityToRPM;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
