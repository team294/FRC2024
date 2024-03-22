// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.SwerveConstants;

/**
 * Class that defines and caches all trajectories that the robot could run.
 * Create one object instance of this class when the robot initializes to build the trajectories. 
 */
public class TrajectoryCache {
    private FileLog log;
   
    private static int trajectoryCount = 27;
    public TrajectoryFacing[] cache = new TrajectoryFacing[trajectoryCount];        // array of trajectories

    public enum TrajectoryType {
        test(0),
        driveToSourceCloseNoteRed(1),
        driveToSourceCloseNoteBlue(2),
        driveToCenterCloseNoteRed(3),
        driveToCenterCloseNoteBlue(4),
        driveToAmpCloseNoteRed(5),
        driveToAmpCloseNoteBlue(6),
        driveAmpNoteToFarNoteRed(7),
        driveAmpNoteToFarNoteBlue(8),
        driveSourceNoteToFarNoteRed(9),
        driveSourceNoteToFarNoteBlue(10),
        driveCenterAmpNoteRed(11),
        driveCenterAmpNoteBlue(12),
        driveFromAmpToCenterRed(13),
        driveFromAmpToCenterBlue(14),
        driveFromAmpNoteToCenterStartRed(15),
        driveFromAmpNoteToCenterStartBlue(16),
        driveFromCenterNoteToCenterStartRed(17),
        driveFromCenterNoteToCenterStartBlue(18),
        driveCenterToNearSourceRed(19),
        driveCenterToNearSourceBlue(20),
        driveFromSourceNearToCenterStartRed(21),
        driveFromSourceNearToCenterStartBlue(22),
        driveFromSourceNearToSourceStartRed(23),
        driveFromSourceNearToSourceStartBlue(24),
        driveFromAmpNearToAmpStartRed(25),
        driveFromAmpNearToAmpStartBlue(26);




        @SuppressWarnings({"MemberName", "PMD.SingularField"})
        public final int value;
        TrajectoryType(int value) { this.value = value; }
    }

    /**
     * A trajectory with initial and final facing for the robot
     */
    public static class TrajectoryFacing {
        public final Rotation2d initialRotation, finalRotation;
        public final Trajectory trajectory;

        /**
         * Creates a trajectory with initial and final robot facing (rotation).
         * <p> Note that the trajectory by itself does *not* contain robot facings.  The Pose2d angles in the
         * trajectory are the direction of the velocity vector.
         * <p> This object adds initial and final facings for the trajectory.
         * @param initialRotation Expected facing (rotation) of robot at beginning of trajectory
         * @param finalRotation Desired facing (rotation) of robot at end of trajectory
         * @param trajectory The trajectory
         */
        public TrajectoryFacing(Rotation2d initialRotation, Rotation2d finalRotation, Trajectory trajectory) {
            this.initialRotation = initialRotation;
            this.finalRotation = finalRotation;
            this.trajectory = trajectory;
        }

        /**
         * Returns the intial robot pose (position and facing) for the robot prior to running the trajectory
         * @return initial Pose2d
         */
        public Pose2d getInitialPose() {
            return new Pose2d( trajectory.getInitialPose().getTranslation(), initialRotation);
        }
    }

    /**
     * Build all trajectories in this.cache[] for trajectory-following commands.
     * @param log
     */
    public TrajectoryCache(FileLog log){
        this.log = log;
        cache[TrajectoryType.test.value] = new TrajectoryFacing(
            new Rotation2d(0),
            new Rotation2d(Math.PI/2),
            calcTrajectory("Test", .4, .4, false,
            new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))),
            List.of(
                new Translation2d(1, -(2 - Math.sqrt(3))),
                new Translation2d(Math.sqrt(2), -(2 - Math.sqrt(2))),
                new Translation2d(Math.sqrt(3), -1)
            ),
            new Pose2d(2, -2, new Rotation2d(Math.toRadians(270)))
            ));

        cache[TrajectoryType.driveToSourceCloseNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Source Start to Close Note Red", .4, .4, false, 
            new Pose2d(1.1, 4.1, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(2.7, 4.1, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToSourceCloseNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Source Start to Close Note Blue", .4, .4, false, 
            new Pose2d(1.1, 4.1, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(2.7, 4.1, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToCenterCloseNoteRed.value] = new TrajectoryFacing(new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Start to near note Red", .4, .4, false, 
            new Pose2d(1.3, 2.663, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(3.1, 2.663, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToCenterCloseNoteBlue.value] = new TrajectoryFacing(new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Start to near note Blue", .4, .4, false, 
            new Pose2d(1.3, 5.57, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(3.1, 5.57, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveToAmpCloseNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Amp Start to Close Note Red", .8, .4, false, 
            new Pose2d(1.1, 1.22, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(3.1, 1.22, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToAmpCloseNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Amp Start to Close Note Blue", .8, .4, false, 
            new Pose2d(1.1, 7.08, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(3.1, 7.08, new Rotation2d(0))
            ));

        
        cache[TrajectoryType.driveAmpNoteToFarNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .8, .4, false,
            new Pose2d(3.1, 1.2, new Rotation2d(Math.toRadians(-30))), 
            List.of(), 
            new Pose2d(8.2, 0.75, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveAmpNoteToFarNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .8, .4, false,
            new Pose2d(3.1, 7, new Rotation2d(Math.toRadians(30))), 
            List.of(), 
            new Pose2d(8.2, 7.45, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveSourceNoteToFarNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .4, .4, false,
            new Pose2d(3.1, 4.1, new Rotation2d(Math.toRadians(-30))), 
            List.of(), 
            new Pose2d(8.2, 7.4, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveSourceNoteToFarNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .4, .4, false,
            new Pose2d(3.1, 4.1, new Rotation2d(Math.toRadians(30))), 
            List.of(), 
            new Pose2d(8.2, 0.8, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveCenterAmpNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Center Start to Amp Note Red", .8, .4, false, 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-33))), 
            List.of(), 
            new Pose2d(3.1, 1.22, new Rotation2d(Math.toRadians(-33)))
            ));

        cache[TrajectoryType.driveCenterAmpNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Center Start to Amp Note Blue", .8, .4, false, 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(33))), 
            List.of(), 
            new Pose2d(3.1, 7.08, new Rotation2d(Math.toRadians(33)))
            ));

        cache[TrajectoryType.driveFromAmpNoteToCenterStartRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(0)), 
            calcTrajectory("Amp Note To Center Start Red", .8, .4, false, 
            new Pose2d(3.1, 1.22, new Rotation2d(Math.toRadians(147))), 
            List.of(), 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(147)))
            ));

        cache[TrajectoryType.driveFromAmpNoteToCenterStartBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(0)), 
            calcTrajectory("Amp Note To Center Start Blue", .8, .4, false, 
            new Pose2d(3.1, 7.08, new Rotation2d(Math.toRadians(-147))), 
            List.of(), 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(-147)))
            ));

        cache[TrajectoryType.driveFromAmpToCenterRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(0)), 
            calcTrajectory("Amp Start to Close Note Red", .8, .4, false, 
            new Pose2d(3.1, 1.6, new Rotation2d(Math.toRadians(147))), 
            List.of(
                new Translation2d(2.2, 2.7)
            ), 
            new Pose2d(3.1, 2.663, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveFromAmpToCenterBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(0)), 
            calcTrajectory("Amp Start to Close Note Blue", .8, .4, false, 
            new Pose2d(3.1, 7.5, new Rotation2d(Math.toRadians(-147))), 
            List.of(
                new Translation2d(2.2, 5.5)
            ), 
            new Pose2d(3.1, 5.57, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveFromCenterNoteToCenterStartRed.value] = new TrajectoryFacing(new Rotation2d(), 
            new Rotation2d(0), 
            calcTrajectory("Center Note to Center start Red", .4, .4, false, 
            new Pose2d(3.1, 2.663, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(180)))
            ));

        cache[TrajectoryType.driveFromCenterNoteToCenterStartBlue.value] = new TrajectoryFacing(new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Note to Center Start Blue", .4, .4, false, 
            new Pose2d(3.1, 5.57, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(180)))
            ));
        
        cache[TrajectoryType.driveCenterToNearSourceRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Source Start to Close Note Red", .4, .4, false, 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(33))), 
            List.of(), 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(33)))
            ));

        cache[TrajectoryType.driveCenterToNearSourceBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Source Start to Close Note Blue", .4, .4, false, 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(-33))), 
            List.of(), 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(-33)))
            ));

        cache[TrajectoryType.driveFromSourceNearToCenterStartRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Source Start to Close Note Red", .4, .4, false, 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(-147))), 
            List.of(), 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-147)))
            ));

        cache[TrajectoryType.driveFromSourceNearToCenterStartBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Source Start to Close Note Blue", .4, .4, false, 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(147))), 
            List.of(), 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(147)))
            ));

        cache[TrajectoryType.driveFromSourceNearToSourceStartRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Source Note to Source Start Red", .4, .4, false, 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.1, 4.1, new Rotation2d(Math.toRadians(180)))
            ));

        cache[TrajectoryType.driveFromSourceNearToSourceStartBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Source Note to Source Start Blue", .4, .4, false, 
            new Pose2d(2.7, 4.1, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.1, 4.1, new Rotation2d(Math.toRadians(180)))
            ));

        cache[TrajectoryType.driveFromAmpNearToAmpStartRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-33)), 
            new Rotation2d(Math.toRadians(-33)), 
            calcTrajectory("Amp Note to Amp Start Red", .4, .4, false, 
            new Pose2d(3.1, 1.22, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.1, 1.22, new Rotation2d(Math.toRadians(180)))
            ));

        cache[TrajectoryType.driveFromAmpNearToAmpStartBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(33)), 
            new Rotation2d(Math.toRadians(33)), 
            calcTrajectory("Amp Note to Amp Start Blue", .4, .4, false, 
            new Pose2d(3.1, 7.08, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.1, 7.08, new Rotation2d(Math.toRadians(180)))
            ));


        
    }


    /**
     * Builds a single trajectory based on the parameters passed in:
     * @param trajName name of the trajectory
     * @param maxVelRatio maximum velocity multiplier between 0 and 1
     * @param maxAccelRatio maximum acceleration multiplier between 0 and 1
     * @param setReversed true = robot drives backwards, false = robot drives forwards
     * @param startPose Pose2d starting position (coordinates and angle)
     * @param interriorWaypoints List of Translation 2d waypoints (just coordinates)
     * @param endPose Pose2d ending position (coordinates and angle)
     * @return trajectory that is generated
     */
    private Trajectory calcTrajectory(String trajName, double maxVelRatio, double maxAccelRatio, 
        boolean setReversed, Pose2d startPose, List<Translation2d> interriorWaypoints, Pose2d endPose) {
		Trajectory trajectory = null;
	
    	try {

			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"maxSpeed", SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				"maxAcceleration", SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio);

			// Create config for trajectory
            TrajectoryConfig config = new TrajectoryConfig(SwerveConstants.kFullSpeedMetersPerSecond * maxVelRatio,
				SwerveConstants.kFullAccelerationMetersPerSecondSquare * maxAccelRatio)
				.setKinematics(DriveConstants.kDriveKinematics)
				.setReversed(setReversed);			// Set to true if robot is running backwards

            // Generate the trajectory
			trajectory = TrajectoryGenerator.generateTrajectory(
				startPose, interriorWaypoints, endPose, config);

			// debug logging
			TrajectoryUtil.dumpTrajectory(trajectory, log);

		} catch (Exception e) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, 
				"ERROR in calcTrajectory", e.toString(),"exception",e);
		}

		if (trajectory != null) {
			log.writeLogEcho(true, "TrajectoryGeneration", trajName, "SUCCESS", true);
		};
	
		return trajectory;
	}

}