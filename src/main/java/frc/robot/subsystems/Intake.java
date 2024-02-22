/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;
import static frc.robot.Constants.*;


public class Intake extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;
  private final TalonFX motor;
  private final CANSparkMax centeringMotor;
  private final TalonFXConfigurator motorConfigurator;
	private TalonFXConfiguration motorConfig;
	private VoltageOut motorVoltageControl = new VoltageOut(0.0);
  private VelocityVoltage motorVelocityControl = new VelocityVoltage(0.0);
  private VelocityVoltage centeringMotorVelocityControl = new VelocityVoltage(0.0);
  private SimpleMotorFeedforward motorFeedforward;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  private double encoderZero = 0.0;     // Zero position for encoder
  private int timeoutMs = 0; // was 30, changed to 0 for testing

  // Velocity control variables
  private double kS, kV, kA;      // Feed forward parameters
  private boolean velocityControlOn = false;    // Is this subsystem using velocity control currently?
  private double measuredRPM = 0.0;             // Current measured speed
  private double setpointRPM = 0.0;             // Current velocity setpoint

  private final StatusSignal<Double> motorSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> motorTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> motorDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> motorStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> motorEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> motorEncoderVelocity;	
	private final StatusSignal<Double> motorVoltage;	

  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOIntakePieceSensor);

  
  public Intake(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    //motor = new WPI_TalonFX(CANPort);
    motor = new TalonFX(Constants.Ports.CANIntake);
    centeringMotor = new CANSparkMax(328, CANSparkLowLevel.MotorType.kBrushless);
    logRotationKey = log.allocateLogRotation();

    motorConfigurator = motor.getConfigurator();
    motorSupplyVoltage = motor.getSupplyVoltage();
	  motorTemp = motor.getDeviceTemp();
	  motorDutyCycle = motor.getDutyCycle();
	  motorStatorCurrent = motor.getStatorCurrent();
	  motorEncoderPosition = motor.getPosition();
	  motorEncoderVelocity = motor.getVelocity();
    motorVoltage = motor.getMotorVoltage();
    
    
    motorConfig = new TalonFXConfiguration();			// Factory default configuration
    motorConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;		// Don't invert motor
		motorConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;          // Boot to coast mode, so robot is easy to push
    motorConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.0;
		motorConfig.ClosedLoopRamps.VoltageClosedLoopRampPeriod = 0.0;
    // set motor configuration
    // motor.configFactoryDefault();
    motor.setInverted(true);
    motor.setNeutralMode(NeutralModeValue.Brake);
    // motor.configPeakOutputForward(1.0);
    // motor.configPeakOutputReverse(-1.0);
    // motor.configNeutralDeadband(0.01);
    // motor.configVoltageCompSaturation(IntakeConstants.compensationVoltage);
    // motor.enableVoltageCompensation(true);
    // motor.configOpenloopRamp(0.05);   //seconds from neutral to full
    // motor.configClosedloopRamp(0.05); //seconds from neutral to full

    // // set sensor configuration
    // motor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, timeoutMs); 
    // motor.setSensorPhase(false);

    zeroEncoder();

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
    motor.setVoltage(voltage);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }


  public void setCenteringMotorVoltage(double voltage) {
    centeringMotor.setVoltage(voltage);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }


  /**
   * sets the percent of the motor, using voltage compensation if turned on
   * @param percent percent
   */
  public void setMotorPercentOutput(double percent){
    //motor.set(ControlMode.PercentOutput, percent);
    motor.setControl(motorVoltageControl.withOutput(percent*IntakeConstants.compensationVoltage));
  
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  public void setCenteringMotorPercentOutput(double percent){
    centeringMotor.set(percent);
  }
  /**
  * Stops the motor
  */
  public void stopMotor(){
    setMotorPercentOutput(0);
    velocityControlOn = false;
    setpointRPM = 0.0;
  }

  public void stopCenteringMotor(){
    setCenteringMotorPercentOutput(0);
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

  public double getCenteringMotorPositionRaw(){
    //return motor.getSelectedSensorPosition(0);
    // Perhaps refresh
    return centeringMotor.getEncoder().getPosition();
  }
  /**
   * Returns the motor position
   * @return position of motor in revolutions
   */
  public double getMotorPosition(){
    return (getMotorPositionRaw() - encoderZero)/IntakeConstants.ticksPerRevolution;
  }
  public double getCenteringMotorPosition(){
    return (getCenteringMotorPositionRaw() - encoderZero)/IntakeConstants.ticksPerRevolution;
  }
    
  /**
	 * Zero the encoder position in software.
	 */
  public void zeroEncoder() {
    encoderZero = getMotorPositionRaw();
  }

  public void zeroCenteringEncoder(){
    encoderZero = getCenteringMotorPositionRaw();

  }
  /**
   * @return velocity of motor in rpm
   */
  public double getMotorVelocity(){
    motorEncoderVelocity.refresh();
    return motorEncoderVelocity.getValueAsDouble();
    // return motor.getSelectedSensorVelocity(0)*IntakeConstants.rawVelocityToRPM;
  }
  public double getCenteringMotorVelocity(){
    return centeringMotor.getEncoder().getVelocity();
    // return motor.getSelectedSensorVelocity(0)*IntakeConstants.rawVelocityToRPM;
  }

  /**
   * Run motor in a velocity closed loop mode.
   * @param motorRPM setPoint RPM
   */
  public void setMotorVelocity(double motorRPM) {
    velocityControlOn = true;
    setpointRPM = motorRPM;
    motor.setControl(motorVelocityControl
        .withVelocity(motorRPM)
        .withFeedForward(motorFeedforward.calculate(motorRPM)));
    //motor.set(ControlMode.Velocity, setpointRPM/IntakeConstants.rawVelocityToRPM,
      //DemandType.ArbitraryFeedForward, kS*Math.signum(setpointRPM) + kV*setpointRPM);
  }

  public void setCenteringMotorVelocity(double centeringMotorRPM) {
    velocityControlOn = true;
    setpointRPM = centeringMotorRPM;
    motor.setControl(centeringMotorVelocityControl
        .withVelocity(centeringMotorRPM)
        .withFeedForward(motorFeedforward.calculate(centeringMotorRPM)));
    //motor.set(ControlMode.Velocity, setpointRPM/IntakeConstants.rawVelocityToRPM,
      //DemandType.ArbitraryFeedForward, kS*Math.signum(setpointRPM) + kV*setpointRPM);
  }

  /**
   * 
   * @return true if piece is in intake
   */
  public boolean isPiecePresent(){
    return !pieceSensor.get();
  }



  @Override
  public void periodic(){
    measuredRPM = getMotorVelocity();
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if(log.isMyLogRotation(logRotationKey)) {
        SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), motorVoltage.refresh().getValueAsDouble());
        SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getMotorPosition());
        SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), measuredRPM);
        SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), motorTemp.refresh().getValueAsDouble());
        SmartDashboard.putBoolean(buildString(subsystemName, " Is Piece Present"), isPiecePresent());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about shooter to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
        log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", motorSupplyVoltage.refresh().getValueAsDouble(),
      "Out Percent", motorDutyCycle.refresh().getValueAsDouble(),
      "Volt", motorVoltage.refresh().getValueAsDouble(), 
      "Amps", motor.getSupplyCurrent(),
      "Temperature", motorTemp.refresh().getValueAsDouble(),
      "Position", getMotorPosition(),
      "Measured RPM", measuredRPM,
      "Setpoint RPM", setpointRPM
    );
  }

  
}