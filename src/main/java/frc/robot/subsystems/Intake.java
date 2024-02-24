/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.VoltageOut;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.Ports;
import frc.robot.utilities.FileLog;
import frc.robot.utilities.Loggable;
import static frc.robot.utilities.StringUtil.*;


public class Intake extends SubsystemBase implements Loggable {
  private final FileLog log;
  private final int logRotationKey;
  private boolean fastLogging = false; // true is enabled to run every cycle; false follows normal logging cycles
  private String subsystemName;    // subsystem name for use in file logging and Shuffleboard

  // Create Neo for centering motor
  private final CANSparkMax centeringMotor = new CANSparkMax(Constants.Ports.CANCenteringMotor, CANSparkLowLevel.MotorType.kBrushless);

  // Create Kraken variables for intake motor
  private final TalonFX intakeMotor = new TalonFX(Constants.Ports.CANIntake);
  private final TalonFXConfigurator intakeConfigurator = intakeMotor.getConfigurator();
	private TalonFXConfiguration intakeConfig;
	private VoltageOut intakeVoltageControl = new VoltageOut(0.0);

  // Create Kraken variables for intake motor
  private final StatusSignal<Double> intakeSupplyVoltage;				// Incoming bus voltage to motor controller, in volts
	private final StatusSignal<Double> intakeTemp;				// Motor temperature, in degC
	private final StatusSignal<Double> intakeDutyCycle;				// Motor duty cycle percent power, -1 to 1
	private final StatusSignal<Double> intakeStatorCurrent;		// Motor stator current, in amps (+=fwd, -=rev)
	private final StatusSignal<Double> intakeEncoderPosition;			// Encoder position, in pinion rotations
	private final StatusSignal<Double> intakeEncoderVelocity;	
	private final StatusSignal<Double> intakeVoltage;

  // Piece sensor inside the intake
  private final DigitalInput pieceSensor = new DigitalInput(Ports.DIOIntakePieceSensor);
  
  /**
   * Creates the intake subsystem
   * @param subsystemName
   * @param log
   */
  public Intake(String subsystemName, FileLog log) {
    this.log = log; // save reference to the fileLog
    this.subsystemName = subsystemName;
    logRotationKey = log.allocateLogRotation();

    // Get signal and sensor objects
    intakeSupplyVoltage = intakeMotor.getSupplyVoltage();
	  intakeTemp = intakeMotor.getDeviceTemp();
	  intakeDutyCycle = intakeMotor.getDutyCycle();
	  intakeStatorCurrent = intakeMotor.getStatorCurrent();
	  intakeEncoderPosition = intakeMotor.getPosition();
	  intakeEncoderVelocity = intakeMotor.getVelocity();
    intakeVoltage = intakeMotor.getMotorVoltage();
    
    // Configure the intake motor
    intakeConfig = new TalonFXConfiguration();			// Factory default configuration
    intakeConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;		// Don't invert motor
		intakeConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;          // Hold piece if we stop the motor
    intakeConfig.Voltage.PeakForwardVoltage = IntakeConstants.compensationVoltage;
    intakeConfig.Voltage.PeakReverseVoltage = -IntakeConstants.compensationVoltage;
    intakeConfig.OpenLoopRamps.VoltageOpenLoopRampPeriod = 0.3;         // # seconds from 0 to full power

    // Set intake motor sensor
    intakeConfig.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;  // use built-in encoder

    // set intake motor configuration
    intakeConfigurator.apply(intakeConfig);

    // configure the centering motor
    centeringMotor.restoreFactoryDefaults();
    centeringMotor.setInverted(false);
    centeringMotor.setIdleMode(IdleMode.kCoast);
    centeringMotor.setOpenLoopRampRate(0.30); //seconds from neutral to full
    centeringMotor.enableVoltageCompensation(IntakeConstants.compensationVoltage);

    stopIntakeMotor();
    stopCenteringMotor();
  }

  /**
   * Returns the name of the subsystem
   */
  public String getName() {
    return subsystemName;
  }

  /**
   * sets the percent of the intake motor, using voltage compensation
   * @param percent percent -1.0 to 1.0 (+ = intake, - = outtake)
   */
  public void setIntakePercentOutput(double percent){
    intakeMotor.setControl(intakeVoltageControl.withOutput(percent*IntakeConstants.compensationVoltage));
  }

  /**
   * Sets the percent output of the centering motor
   * @param percent percent -1.0 to 1.0 (+ = intake, - = outtake)
   */
  public void setCenteringMotorPercentOutput(double percent){
    centeringMotor.set(percent);
  }

  /**
  * Stops the intake motor
  */
  public void stopIntakeMotor(){
    setIntakePercentOutput(0);
  }

  /**
   * Stops the centering motor
   */
  public void stopCenteringMotor(){
    setCenteringMotorPercentOutput(0);
  }

  /**
   * Returns the intake motor position
   * @return position of motor in revolutions
   */
  public double getIntakePosition(){
    intakeEncoderPosition.refresh();
    return intakeEncoderPosition.getValueAsDouble();
  }

  /**
   * Returns the centering motor position
   * @return position of the motor in revolutions
   */
  public double getCenteringMotorPosition(){
    return centeringMotor.getEncoder().getPosition();
  }
    
  /**
   * Get the velocity of the intake motor
   * @return velocity of motor in RPM
   */
  public double getIntakeVelocity(){
    intakeEncoderVelocity.refresh();
    return intakeEncoderVelocity.getValueAsDouble() * 60.0;
  }

  /**
   * Get the velocity of the centering motor
   * @return velocity of centering motor in RPM
   */
  public double getCenteringMotorVelocity(){
    return centeringMotor.getEncoder().getVelocity();
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
    if(fastLogging || log.isMyLogRotation(logRotationKey)) {
      updateLog(false);
    }

    if(log.isMyLogRotation(logRotationKey)) {
        SmartDashboard.putNumber(buildString(subsystemName, " Voltage"), intakeVoltage.refresh().getValueAsDouble());
        SmartDashboard.putNumber(buildString(subsystemName, " Position Rev"), getIntakePosition());
        SmartDashboard.putNumber(buildString(subsystemName, " Velocity RPM"), getIntakeVelocity());
        SmartDashboard.putNumber(buildString(subsystemName, " Temperature C"), intakeTemp.refresh().getValueAsDouble());
        SmartDashboard.putNumber("Centering Percent", centeringMotor.getAppliedOutput());
        SmartDashboard.putNumber("Centering Position Rev", getCenteringMotorPosition());
        SmartDashboard.putNumber("Centering Velocity RPM", getCenteringMotorVelocity());
        SmartDashboard.putNumber("Centering Temperature C", centeringMotor.getMotorTemperature());
        SmartDashboard.putBoolean(buildString(subsystemName, " Is Piece Present"), isPiecePresent());
    }
  }

  @Override
  public void enableFastLogging(boolean enabled) {
    fastLogging = enabled;
  }

  /**
   * Write information about intake to fileLog.
   * @param logWhenDisabled true = log when disabled, false = discard the string
   */
	public void updateLog(boolean logWhenDisabled) {
        log.writeLog(logWhenDisabled, subsystemName, "Update Variables",  
      "Bus Volt", intakeSupplyVoltage.refresh().getValueAsDouble(),
      "Intake Percent", intakeDutyCycle.refresh().getValueAsDouble(),
      "Centering Percent", centeringMotor.getAppliedOutput(),
      "Intake Amps", intakeStatorCurrent.refresh().getValueAsDouble(),
      "Centering Amps", centeringMotor.getOutputCurrent(), 
      "Intake Temperature", intakeTemp.refresh().getValueAsDouble(),
      "Centering Temperature", centeringMotor.getMotorTemperature(),
      "Intake Position", getIntakePosition(),
      "Centering Position", getCenteringMotorPosition(),
      "Intake RPM", getIntakeVelocity(),
      "Centering RPM", getCenteringMotorVelocity()
    );
  }

  
}