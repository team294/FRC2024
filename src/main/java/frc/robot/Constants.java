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
import edu.wpi.first.wpilibj.util.Color;
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
      // public static final int CANPneumaticHub = 1;

      public static final String CANivoreBus = "CANivore";

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

      public static final int CANShooterTop = 13;
      public static final int CANShooterBottom = 14;

      public static final int CANFeeder = 15;

      public static final int CANIntake = 16;
      public static final int CANCenteringMotor = 17;

      public static final int CANPigeonGyro = 18;

      public static final int CANWrist1 = 19;
      public static final int CANWrist2 = 20;

      public static final int CANLED = 21;

      // Digital IO ports
      public static final int DIOFeederPieceSensor = 0;
      public static final int DIOWristRevThroughBoreEncoder = 1;
      public static final int DIOIntakePieceSensor = 9;
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
      // Drivebase adjustment for path-of-wheel diameter when turning in place
      private static final double DrivetrainAdjustmentFactor = 1.000;       // TODO CALIBRATE
      // left to right distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_TRACKWIDTH_METERS = 0.61595 * DrivetrainAdjustmentFactor;      // 0.61595m CALIBRATED.  Competition bot CAD = 24.25" = 0.61595m. 80% bot CAD = 0.60325m, calibrated = 0.626m.
      // front-back distance between the drivetrain wheels; should be measured from center to center
      public static final double DRIVETRAIN_WHEELBASE_METERS = 0.52705 * DrivetrainAdjustmentFactor;       // 0.52705m CALIBRATED.  Competition bot CAD = 20.75" = 0.52705m.  80% bot CAD = 0.60325m, calibrated = 0.626m.

    }

    public static final class SwerveConstants {
        // Encoder calibration to meters travelled or wheel facing degrees
      public static final double kEncoderCPR = 1.0;                // CALIBRATED = 1.0.  Encoder counts per revolution of motor pinion gear
      public static final double kDriveGearRatio = (5.903 / 1.0);      // 2024=Modified L2.  CALIBRATED.   Mk4i = 8.14:1 (L1-std gears), 6.75:1 (L2-fast gears), 5.903 (modified L2 16-tooth gear).  
      public static final double kTurningGearRatio = (150.0/7.0 / 1.0); // CALIBRATED = 150.0/7.0.  Mk4i = 150/7 : 1
      public static final double kWheelDiameterMeters = 0.1003 * 1.001; // 1.001 adjustment CALIBRATED.  Colson wheel = nominal 4" diameter, actual 3.95" = 0.1003m.  80% bot calibrated = 0.1013m.
      public static final double kDriveEncoderMetersPerTick = (kWheelDiameterMeters * Math.PI) / kEncoderCPR / kDriveGearRatio;
      public static final double kTurningEncoderDegreesPerTick = 360.0/kEncoderCPR / kTurningGearRatio;

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
      public static final double kFullAccelerationMetersPerSecondSquare = 0.9 * kMaxAccelerationMetersPerSecondSquare;
      public static final double kNominalAccelerationMetersPerSecondSquare = 3.5; // TODO value from last year
      public static final double kMaxRetractingAccelerationMetersPerSecondSquare = 2; // TODO value from last year - not used in code currently
      public static final double kMaxTurningRadiansPerSecond = 11.0;  // TODO NOT CALIBRATED
      public static final double kNominalTurningRadiansPerSecond = Math.PI;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = 35.0;            // TODO NOT CALIBRATED - not used in code currently
      public static final double kNominalAngularAccelerationRadiansPerSecondSquared = Math.PI;
      public static final double kVDriveAvg = 0.1740; // init cal done.  TODO Calibrate.  0.2034 from 2023 robot.  In % output per meters per second.
      public static final double kVmFL = 1.0182;      // init cal done.  TODO Calibrate.  kV modifier for FL drive motor
      public static final double kVmFR = 0.9826;      // init cal done.  TODO Calibrate.  kV modifier for FR drive motor
      public static final double kVmBL = 1.0102;      // init cal done.  TODO Calibrate.  kV modifier for BL drive motor
      public static final double kVmBR = 0.9889;      // init cal done.  TODO Calibrate.  kV modifier for BR drive motor

      public static final double kADrive = 0.0;
      public static final double kADriveToPose = 0.050;  // formerly 0.060  CALIBRATED.  In % output per meters per second squared.
      public static final double kSDrive = 0.0080; // init cal done.  formerly 0.0255, CALIBRATED.  In % output.
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
      public static double offsetAngleFrontLeftMotor = 0; // 110.7
      public static double offsetAngleFrontRightMotor = 0; // 44.6
      public static double offsetAngleBackLeftMotor = 0; // -67.1
      public static double offsetAngleBackRightMotor = 0; // 152.1

        // Driving constants to cap acceleration
      public static final double maxAccelerationRate = 10.0;         // m/s^2
      public static final double maxAccelerationRateY = 5.0;          // m/s^2
      public static final double maxAccelerationRateAtScoreMid = 7;           // m/s^2
      public static final double maxAccelerationRateBetweenScoreMidAndHigh = 6.0;        // m/s^2
      public static final double maxAccelerationRateWithElevatorUp = 1.5;           // m/s^2
      public static final double maxRotationRateWithElevatorUp = 0.8;     // rad/sec

      public static final double kPJoystickThetaController = 3; // Theta kp value for joystick in rad/sec
    }

    public static final class ShooterConstants {
      public static final double compensationVoltage = 12.0;
      public static final double ticksPerRevolution = 1.0;
      public static final double shooterGearRatio = 1.0;  //(44.0/20.0) * (24.0/18.0);   // From CAD, shooter gears from motor to wheels = 44:20 then 24:18.  Turn ratio from motor pinion to shooter wheels.

      // PIDSVA for Top Shooter motor
      public static final double ShooterTopkP = 0.5;          // TODO calibrate.  kP = (desired-output-volts) / (error-in-encoder-rps)
      public static final double ShooterTopkI = 0.00;         // kI = (desired-output-volts) / (error-in-encoder-rps * s)
      public static final double ShooterTopkD = 0.0;          // kD = (desired-output-volts) / (error-in-encoder-rps/s)
      public static final double ShooterTopkS = 0.061256;     // kS = (desired-output-volts)
      public static final double ShooterTopkV = 0.118681;     // kV = (desired-output-volts) / (target-velocity-in-encoder-rps)
      public static final double ShooterTopkA = 0.0;          // kA = (desired-output-volts) / (target-accel-in-encoder-rot/sec^2)
      
      //PIDSVA for Bottom Shooter motor
      public static final double ShooterBottomkP = 0.5;
      public static final double ShooterBottomkI = 0.00;
      public static final double ShooterBottomkD = 0.0;
      public static final double ShooterBottomkS = 0.061256;  // V; old: 0.004, new: 0.125
      public static final double ShooterBottomkV = 0.118681;  // V * s / dist; old: 0.000155, new: 0.129166,
      public static final double ShooterBottomkA = 0.0;

      /*
        Volt  RPS 
        0.77	5.9453125
        1.41	11.41796875
        2.06	16.814453125
      */

      public static final double velocityErrorTolerance = 100;
      public static final double shooterPercent = 0.25;
      public static final double shooterVelocity = 2500;
    }

    public static final class FeederConstants {
      public static final double compensationVoltage = 12.0;
      public static final double ticksPerRevolution = 1.0;
      public static final double feederGearRatio = 1.0; //(14.0/28.0) * (16.0/32.0);   // From CAD, feeder gears from motor to wheels = 14:28 then 16:32.  Turn ratio from motor pinion to feeder wheels.
      
      //PIDSVA for Feeder
      public static final double kP = 0.5;
      public static final double kI = 0.00;
      public static final double kD = 0.0;
      public static final double kS = 0.061256; // V; old: 0.004, new: 0.125
      public static final double kV = 0.118681; // V * s / dist; old: 0.000155, new: 0.129166,
      public static final double kA = 0.0;

      public static final double feederPercent = 0.2;
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

    public static final class WristConstants {
      public static final double kEncoderCPR = 1.0;                // CALIBRATED = 1.  Encoder counts per revolution of FalconFX motor pinion gear
      public static final double kWristGearRatio = (75.0 / 1.0);   // From CAD, should be 75:1.  Gear reduction ratio between motor pinion and gear driving the wrist (planetary and chain gears)
      public static final double kWristDegreesPerRotation =  360.0 / kEncoderCPR / kWristGearRatio;      // CALIBRATED (fudge factor was 0.9726 last year)

      public static final double kRevEncoderGearRatio = (3.0 / 1.0);   // From CAD, should be 3:1.  Gear reduction ratio between Rev Thru-Bore encoder and gear driving the wrist (chain/gears)
      
      public static final double voltageCompSaturation = 12.0;
      public static final double maxUncalibratedPercentOutput = 0.1;     // CALIBRATED
      public static final double maxPercentOutput = 0.4;          // CALIBRATED

      // Update the REV through bore encoder offset angle in RobotPreferences (in Shuffleboard), not in this code!
      // After updating in RobotPreferences, you will need to re-start the robot code for the changes to take effect.
      // When calibrating offset, 0 deg should be with the CG of the wrist horizontal facing away from the robot,
      // and -90 deg is with the CG of the wrist resting downward.
      public static double revEncoderOffsetAngleWrist = 0;    // 5.0 deg (was 69.0 deg before changing wrist chain)  CALIBRATED

      public static final double kP = 0.5;   // Calc 0.72 from 2023 TODO CALIBRATE. kP value (0.03).  kP = (desired-output-volts) / (error-in-encoder-rotations)
      public static final double kI = 0.0; 
      public static final double kD = 0.0; 
      public static final double kG = 0.0;   // 0.1 initially, TODO CALIBRATE.  2023 was 0.03.  Feed foward percent-out to add to hold arm horizontal (0 deg)

      // Wrist regions
      public enum WristRegion {
          // backFar,        // In the wrist backFar region, the elevator must be in the bottom region (not allowed to go to elevator main or low regions).
          // backMid,        // In the wrist backMid region, the elevator may be in any elevator region.
          // down,           // Wrist pointed down, the elevator must be in the main region.
          // back,           // In the wrist back region, the elevator must be in the bottom region (not allowed to go to elevator main).
          main,           // In the wrist main region, the elevator may be in any elevator region.
          uncalibrated    // Unknown region, wrist is not calibrated
      } 
      // Wrist region boundaries
      // public static final double boundBackFarMid = -119.0;      // Boundary between backFar and backMid regions.  CALIBRATED
      // public static final double boundBackMidDown = -116.0;      // Boundary between backMid and down regions.  CALIBRATED
      // public static final double boundDownMain = -91.0;      // Boundary between down and main regions.  CALIBRATED
      // public static final double boundDownMidpoint = (boundBackMidDown+boundDownMain)/2.0;      // Midpoint in down region
      // public static final double boundBackMain = -120.0;      // Boundary between back and main regions.  CALIBRATED

      // Wrist pre-defined angles (in degrees)
      // 0 degrees = horizontal (in front of robot) relative to wrist center of gravity
      // -90 degrees = vertical = wrist is hanging "down" naturally due to gravity
      public enum WristAngle {
          lowerLimit(-3.0),      // CALIBRATED
          upperLimit(105.0);       // CALIBRATED

          @SuppressWarnings({"MemberName", "PMD.SingularField"})
          public final double value;
          WristAngle(double value) { this.value = value; }
      }
    }

    public static final class IntakeConstants {
      public static final double compensationVoltage = 12.0;                      // voltage compensation on motor

      public static final double intakePercent = 0.6;
      public static final double centeringPercent = 0.3; // Need to calibrate, using talon instead of neo
    }

    /** Colors for the LEDs based on different robot states (see BCRRobotState) */
    public enum BCRColor {
      IDLE_NO_PIECE(255, 255 ,255), // White
      IDLE_WITH_PIECE(255, 30, 0), // Orange
      INTAKE_TO_FEEDER(255, 0, 20), // Red
      SHOOTING(0, 255, 100); // Green

      public final int r;
      public final int g;
      public final int b;
      BCRColor(int r, int g, int b) {
        this.r = r;
        this.g = g;
        this.b = b;
      }
    }

    public static final class LEDConstants {
        public enum LEDSegment {
            CANdle(0, 8, null),
            Strip1(8, 60, null),
            Full(0, 68, null);

            public final int index, count;
            public Color[][] animation;
            public int state;
            LEDSegment(int index, int count, Color[][] animation){ this.index = index; this.count = count; this.animation = animation;state = 0;}
        }

        // public static final double NumLEDs = 68;
    }

}