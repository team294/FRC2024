// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;


import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.utilities.TrapezoidProfileBCR;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public enum CoordType {
        kRelative,              // Relative to current robot location/facing
        kAbsolute,              // Absolute field coordinates, don't reset robot pose
        kAbsoluteResetPose,     // Absolute field coordinates, reset robot pose always
        kAbsoluteResetPoseTol;  // Absolute field coordinates, reset robot pose if robot is not close to specified position
    }

    /**
     * Options to select driving stopping types.
     */
    public enum StopType {
        kNoStop,
        kCoast,
        kBrake;
    }

    public static final class Ports{
      public static final int CANPneumaticHub = 1;

      public static final int CANDriveFrontLeftMotor = 1;
      public static final int CANDriveFrontRightMotor = 2;
      public static final int CANDriveBackLeftMotor = 3;
      public static final int CANDriveBackRightMotor = 4;

      public static final int CANDriveTurnFrontLeftMotor = 5;
      public static final int CANDriveTurnFrontRightMotor = 6;
      public static final int CANDriveTurnBackLeftMotor = 7;
      public static final int CANDriveTurnBackRightMotor = 8;

      // Note:  Remote sensors accessed by a Talon FX (Falcon 500) must have a CAN ID of 15 or less. See errata
      // in CTRE documentation "Talon FX Remote Filter Device ID Must be 15 or Less" for more details.
      // This applies to the turn encoders, which are used as remote sensors for the turn motors.
      public static final int CANTurnEncoderFrontLeft = 9;
      public static final int CANTurnEncoderFrontRight = 10;
      public static final int CANTurnEncoderBackLeft = 11;
      public static final int CANTurnEncoderBackRight = 12;

      public static final int CANShooter1 = 13;
      public static final int CANShooter2 = 14;

      public static final int CANFeeder = 15;

      public static final int CANIntake = 16;

      public static final int DIOIntakePieceSensor = 0;

    }

    public static final class OIConstants {
      //Ports from last year
      public static final int usbXboxController = 0;
      public static final int usbLeftJoystick = 1;
      public static final int usbRightJoystick = 2;
      public static final int usbCoPanel = 3;

      public static final double joystickDeadband = 0.01;
      public static final double manualElevatorDeadband = 0.1;
      public static final double manualWristDeadband = 0.1;
    }

    public static final class RobotDimensions {
      //left to right distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.58721;      // TODO NOT CALIBRATED
      //front-back distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_WHEELBASE_METERS = 0.58721;       // TODO NOT CALIBRATED

    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
      public static final double kEncoderCPR = 1.0;                // CALIBRATED = 1.  Encoder counts per revolution of motor pinion gear
      public static final double kDriveGearRatio = (8.14 / 1.0);      // TODO NOT CALIBRATED.   Mk4i = 8.14:1 (L1-std gears).  Mk4i = 6.75:1 (L2-fast gears)
      public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED = 150.0/7.0.  Mk4i = 150/7 : 1
      public static final double kWheelDiameterMeters = 0.09712;        // TODO NOT CALIBRATED
      // public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
      // public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;
      public static final double kDriveEncoderMetersPerRotation = (kWheelDiameterMeters * Math.PI) / kDriveGearRatio;
      public static final double kTurningEncoderDegreesPerRotation = 360.0/ kTurningGearRatio;

      // Robot calibration for feed-forward and max speeds
      public static final double voltageCompSaturation = 12.0;
      // Max speed is used to keep each motor from maxing out, which preserves ratio between motors 
      // and ensures that the robot travels in the requested direction.  So, use min value of all 4 motors,
      // and further derate (initial test by 5%) to account for some battery droop under heavy loads.
      // Max speed measured values x/x/2024:  All 4 motors are between 4.6 an 4.7 meters/sec.  So use 4.5 as a conservative value
      public static final double kMaxSpeedMetersPerSecond = 4.5;          // TODO NOT CALIBRATED
      public static final double kFullSpeedMetersPerSecond = 0.95*kMaxSpeedMetersPerSecond;
      public static final double kNominalSpeedMetersPerSecond = 0.5*kMaxSpeedMetersPerSecond;
      // Max acceleration measured x/x/2024 (with full robot weight):  Average ~11 m/sec^2.  Keep value at 10.0 for now.
      public static final double kMaxAccelerationMetersPerSecondSquare = 10; // TODO NOT CALIBRATED
      public static final double kFullAccelerationMetersPerSecondSquare = 0.9*kMaxAccelerationMetersPerSecondSquare;
      public static final double kNominalAccelerationMetersPerSecondSquare = 3.5; // TODO value from last year
      public static final double kMaxRetractingAccelerationMetersPerSecondSquare = 2; // TODO value from last year
      public static final double kMaxTurningRadiansPerSecond = 11.0;  // TODO NOT CALIBRATED
      public static final double kNominalTurningRadiansPerSecond = Math.PI;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;            // TODO NOT CALIBRATED
      public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;
      public static final double kVDrive = 0.2034; // TODO NOT CALIBRATED.  In % output per meters per second.
      public static final double kADrive = 0.0;
      public static final double kADriveToPose = 0.060;  // TODO NOT CALIBRATED.  In % output per meters per second squared.
      public static final double kSDrive = 0.016; // TODO NOT CALIBRATED.  In % output.
    }

    public static final class DriveConstants {
      // The locations of the wheels relative to the physical center of the robot, in meters.
      // X: + = forward.  Y: + = to the left
      // The order in which you pass in the wheel locations is the same order that
      // you will receive the module states when performing inverse kinematics. It is also expected that
      // you pass in the module states in the same order when calling the forward kinematics methods.
      // 0 = FrontLeft, 1 = FrontRight, 2 = BackLeft, 3 = BackRight
      public static final SwerveDriveKinematics kDriveKinematics =
            new SwerveDriveKinematics(
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2),
                new Translation2d(-RobotDimensions.DRIVETRAIN_WHEELBASE_METERS / 2, -RobotDimensions.DRIVETRAIN_TRACKWIDTH_METERS / 2));

      // Update the offset angles in RobotPreferences (in Shuffleboard), not in this code!
      // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
      // When calibrating offset, set the wheels to zero degrees with the bevel gear facing to the right
      public static double offsetAngleFrontLeftMotor = 0; // -72
      public static double offsetAngleFrontRightMotor = 0; // -157
      public static double offsetAngleBackLeftMotor = 0; // 44.5
      public static double offsetAngleBackRightMotor = 0; // -82

        // Driving constants to cap acceleration
      public static final double maxAccelerationRate = 10.0;         // m/s^2
      public static final double maxAccelerationRateY = 5.0;          // m/s^2
      public static final double maxAccelerationRateAtScoreMid = 7;           // m/s^2
      public static final double maxAccelerationRateBetweenScoreMidAndHigh = 6.0;        // m/s^2
      public static final double maxAccelerationRateWithElevatorUp = 1.5;           // m/s^2
      public static final double maxRotationRateWithElevatorUp = 0.8;     // rad/sec


      public static final double kPJoystickThetaController = 3; // Theta kp value for joystick in rad/sec

      // Drivetrain DrivingMotorPID
      public static final class DrivingMotorPID {
        public static final double kP = 0;     // Was 6e-5?
        public static final double kI = 0;
        public static final double kD = 0; 
        public static final double kIz = 0; 
        public static final double kFF = 0;  // was 0.000015?
        public static final double kMaxOutput = 1; 
        public static final double kMinOutput = -1;
      }

      // Drivetrain TurningMotorPID
      public static final class TurningMotorPID {
          public static final double kP = 0;       // Was 0.3?
          public static final double kI = 0;     // Was 1e-4?
          public static final double kD = 0;        // Was 1?
          public static final double kIz = 0;
          public static final double kFF = 0;
          public static final double kMaxOutput = 1;
          public static final double kMinOutput = -1;
      } 
    }

    public static final class ShooterConstants {
      // TODO: add all necessary constants
      public static final double compensationVoltage = 12.0; // TODO: check value
      public static final double ticksPerRevolution = 2048.0; // TODO: check value
      // PIDSVA
      public static final double kP = 0.02;
      public static final double kI = 0.005;
      public static final double kD = 0.0;
      public static final double kS = 0.004;
      public static final double kV = 0.000155;
      public static final double kA = 0.0;

      public static final double shooterPercent = 0.25;

      //Feeder Constants
      public static final double feederPercent = 0.3;
    }

    public static final class TrajectoryConstants {


      // Max error for robot rotation
      public static final double maxThetaErrorDegrees = 1.0;
      public static final double maxPositionErrorMeters = 0.04; // 1.6 inches

      // Max error for interim positions (not final)
      public static final double interimThetaErrorDegrees = 2.0;        
      public static final double interimPositionErrorMeters = 0.20; // 8 inches

      // Feedback terms for holonomic drive controllers

      // X-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)
      public static final double kPXController = 1;

      // Y-velocity controller:  kp.  Units = (meters/sec of velocity) / (meters of position error)  
       public static final double kPYController = 1; 

      public static final double kPThetaController = 3;   // Theta-velocity controller:  kp.  Units = (rad/sec of velocity) / (radians of angle error)

      public static final TrajectoryConfig swerveTrajectoryConfig =
            new TrajectoryConfig(
                    SwerveConstants.kNominalSpeedMetersPerSecond,
                    SwerveConstants.kNominalAccelerationMetersPerSecondSquare)
                .setKinematics(DriveConstants.kDriveKinematics);

        /* Constraint for the motion profilied robot angle controller */
      public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
      new TrapezoidProfile.Constraints(
            SwerveConstants.kNominalTurningRadiansPerSecond, SwerveConstants.kNominalAngularAccelerationRadiansPerSecondSquared);

        /* Constraint for the DriveToPose motion profile for distance being travelled */
      public static final TrapezoidProfileBCR.Constraints kDriveProfileConstraints =
      new TrapezoidProfileBCR.Constraints(
            SwerveConstants.kNominalSpeedMetersPerSecond, SwerveConstants.kNominalAccelerationMetersPerSecondSquare);
    }

    public static class FieldConstants {
      public static final double length = Units.feetToMeters(54);
      public static final double width = Units.feetToMeters(27);
    }

    public static class VisionConstants {
      //TODO NEED TO CALIBRATE
      public static final Transform3d robotToCam =
                new Transform3d(
                    // new Translation3d(Units.inchesToMeters(6.0), 0.0, Units.inchesToMeters(30.5)),       Changed in B3
                    new Translation3d(Units.inchesToMeters(0), 0, Units.inchesToMeters(0)),
                    new Rotation3d(0, Units.degreesToRadians(0), 0)); // Cam mounted facing forward in center of robot
        public static final String cameraName = "CenterCamera";
    }

    public static final class IntakeConstants {
      public static final double compensationVoltage = 12.0;                      // voltage compensation on motor
      public static final double ticksPerRevolution = 2048.0;                     // Divide by this to convert raw ticks to revolutions
      public static final double rawVelocityToRPM = 600.0 / ticksPerRevolution;   // Multiply by this to convert raw velocity (ticksPer100ms) to RPM

      public static final double intakePercent = 0.3;
  }

}