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
   
    private static int trajectoryCount = 75;
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
        driveFromCenterStartToEndCenterAutoRed(23),
        driveFromCenterStartToEndCenterAutoBlue(24),
        driveFromSourceNoteToSourceStartRed(25),
        driveFromSourceNoteToSourceStartBlue(26),
        driveCenterStartToSourceNearRed(27),
        driveCenterStartToSourceNearBlue(28),
        driveFromSourceNoteToCenterNoteRed(29),
        driveFromSourceNoteToCenterNoteBlue(30),
        driveFromCenterNoteToAmpNoteRed(31),
        driveFromCenterNoteToAmpNoteBlue(32),
        driveSourceOutsideNotesRed(33),
        driveSourceOutsideNotestoCenterNoteRed(34),
        driveCenterNotetoOutsideStageRed(35),
        driveOutsideStageLeftCenterNoteRed(36),
        driveLeftCenterNotetoOutsideStageRed(37),
        driveSourceOutsideNotesBlue(38),
        driveSourceOutsideNotestoCenterNoteBlue(39),
        driveCenterNotetoOutsideStageBlue(40),
        driveOutsideStageLeftCenterNoteBlue(41),
        driveLeftCenterNotetoOutsideStageBlue(42),
        driveFromAmpFarToShootingPosRed(43),
        driveFromAmpFarToShootingPosBlue(44),
        driveAmpToFarCenterRed(45),
        driveFarCenterNoteToPodiumShotRed(46),
        drivePodiumShotToNextCenterNoteRed(47),
        driveNextCenterNotetoPodiumShotRed(48),
        drivePodiumShotToCenterNoteRed(49),
        driveCenterNotetoPodiumShotRed(50),
        driveAmpToFarCenterBlue(51),
        driveFarCenterNoteToPodiumShotBlue(52),
        drivePodiumShotToNextCenterNoteBlue(53),
        driveNextCenterNotetoPodiumShotBlue(54),
        drivePodiumShotToCenterNoteBlue(55),
        driveCenterNotetoPodiumShotBlue(56),
        driveAmpToSecondFarCenterRed(57),
        driveAmpToSecondFarCenterBlue(58),
        driveNextCenterNoteToCenterNoteRed(59),
        driveNextCenterNoteToCenterNoteBlue(60),
        driveFirstCenterAmpToNextCenterNoteRed(61),
        driveFirstCenterAmpToNextCenterNoteBlue(62),
        driveFromAmpNoteToSecondCenterRed(63),
        driveFromCenterSecondToScorePosRed(64),
        driveFromAmpNoteToSecondCenterBlue(65),
        driveFromCenterSecondToScorePosBlue(66),
        driveFromAmpNoteToSecondCenter(67),
        driveFromCenterSecondToScorePos(68),
        driveSourceNextNoteToCenterNoteRightRed(69),
        driveSourceNextNoteToCenterNoteRightBlue(70),
        drivePodiumShotToCenterRightNoteRed(71),
        drivePodiumShotToCenterRightNoteBlue(72),
        driveCenterRightNoteToPodiumShotRed(73),
        driveCenterRightNoteToPodiumShotBlue(74);


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
            new Rotation2d(Math.toRadians(54)), 
            new Rotation2d(0), 
            calcTrajectory("Source Start to Close Note Red", .4, .4, false, 
            new Pose2d(1.1, 3.463, new Rotation2d(Math.toRadians(10))), 
            List.of(), 
            new Pose2d(2.8, 3.963, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToSourceCloseNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-54)), 
            new Rotation2d(0), 
            calcTrajectory("Source Start to Close Note Blue", .4, .4, false, 
            new Pose2d(1.3, 4.6, new Rotation2d(Math.toRadians(-10))), 
            List.of(), 
            new Pose2d(2.8, 4.1, new Rotation2d(0))
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
            new Rotation2d(Math.toRadians(-60)), 
            new Rotation2d(Math.toRadians(-26)), 
            calcTrajectory("Amp Start to Close Note Red", .8, .4, false, 
            new Pose2d(0.7, 1.3, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(3.0, 0.8, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveToAmpCloseNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(60)), 
            new Rotation2d(Math.toRadians(26)), 
            calcTrajectory("Amp Start to Close Note Blue", .8, .4, false, 
            new Pose2d(0.7, 6.5, new Rotation2d(Math.toRadians(0))), 
            List.of(), 
            new Pose2d(3.0, 7.4, new Rotation2d(0))
            ));

        
        cache[TrajectoryType.driveAmpNoteToFarNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-25)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .8, .8, false,
            new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(-30))), 
            List.of(), 
            new Pose2d(7.8, 0.75, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveAmpNoteToFarNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(25)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Blue", .8, .8, false,
            new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(30))), 
            List.of(), 
            new Pose2d(7.8, 7.35, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveSourceNoteToFarNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .4, .4, false,
            new Pose2d(3.0, 4.1, new Rotation2d(Math.toRadians(-30))), 
            List.of(), 
            new Pose2d(8.2, 7.4, new Rotation2d(0))
            ));
        
        cache[TrajectoryType.driveSourceNoteToFarNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Far Note Red", .4, .4, false,
            new Pose2d(3.0, 4.1, new Rotation2d(Math.toRadians(30))), 
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

        cache[TrajectoryType.driveFromCenterStartToEndCenterAutoRed.value] = new TrajectoryFacing(new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Start to near note Red", .8, .4, false, 
            new Pose2d(1.3, 2.663, new Rotation2d(Math.toRadians(-17))), 
            List.of(), 
            new Pose2d(7, 1.22, new Rotation2d(Math.toRadians(-17)))
            ));

        cache[TrajectoryType.driveFromCenterStartToEndCenterAutoBlue.value] = new TrajectoryFacing(new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Start to near note Blue", .8, .4, false, 
            new Pose2d(1.3, 5.57, new Rotation2d(Math.toRadians(17))), 
            List.of(), 
            new Pose2d(7, 7.08, new Rotation2d(Math.toRadians(17)))
            ));

        cache[TrajectoryType.driveFromSourceNoteToSourceStartRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(54)), 
            calcTrajectory("Source Note to Source Start Red", .8, .4, false, 
            new Pose2d(2.8, 3.963, new Rotation2d(Math.toRadians(180))),
            List.of(), 
            new Pose2d(1.1, 3.463, new Rotation2d(Math.toRadians(-170)))
            ));

        cache[TrajectoryType.driveFromSourceNoteToSourceStartBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-54)), 
            calcTrajectory("Source Note to Source Start Blue", .8, .4, false, 
            new Pose2d(2.8, 4.1, new Rotation2d(Math.toRadians(180))), 
            List.of(), 
            new Pose2d(1.1, 4.6, new Rotation2d(Math.toRadians(170)))
            ));
        
        cache[TrajectoryType.driveCenterStartToSourceNearRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(23)), 
            calcTrajectory("Center Start to Source note Red", .8, .8, false, 
            new Pose2d(0.4, 2.65, new Rotation2d(Math.toRadians(23))), 
            List.of(), 
            new Pose2d(1.7, 3.95, new Rotation2d(Math.toRadians(23)))
            ));
        
        cache[TrajectoryType.driveCenterStartToSourceNearBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-25)), 
            calcTrajectory("Center Start to Source note Blue", .8, .8, false, 
            new Pose2d(0.4, 5.55, new Rotation2d(Math.toRadians(-23))), 
            List.of(), 
            new Pose2d(1.7, 4.25, new Rotation2d(Math.toRadians(-23)))
            ));

        cache[TrajectoryType.driveFromSourceNoteToCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(23)), 
            new Rotation2d(0), 
            calcTrajectory("Source Note to Center note Red", .8, .8, false, 
            new Pose2d(1.7, 3.95, new Rotation2d(Math.toRadians(-160))),
            List.of(new Translation2d(1.6, 2.65)), 
            new Pose2d(2.0, 2.65, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveFromSourceNoteToCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-25)), 
            new Rotation2d(0), 
            calcTrajectory("Source Note to Center note Blue", .8, .8, false, 
            new Pose2d(1.7, 4.25, new Rotation2d(Math.toRadians(160))),
            List.of(new Translation2d(1.6, 5.45)), 
            new Pose2d(2.0, 5.45, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveFromCenterNoteToAmpNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-25)), 
            calcTrajectory("Center Note to Amp note Red", .8, .8, false, 
            new Pose2d(2.0, 2.65, new Rotation2d(Math.toRadians(-160))),
            List.of(new Translation2d(1.6, 1.25)), 
            new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(0)))
            ));
        
        cache[TrajectoryType.driveFromCenterNoteToAmpNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(25)), 
            calcTrajectory("Center Note  to Amp note Blue", .8, .8, false, 
            new Pose2d(2.0, 5.45, new Rotation2d(Math.toRadians(160))),
            List.of(new Translation2d(1.6, 6.85)), 
            new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(0)))
            ));
        
        cache[TrajectoryType.driveSourceOutsideNotesRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(54)), 
            new Rotation2d(0), 
            calcTrajectory("Source to next to note Red", .9, .9, false, 
            new Pose2d(0.8, 3.73, new Rotation2d(Math.toRadians(54))), 
            List.of(new Translation2d(1.2, 4.3)), 
            new Pose2d(3.5, 5.705, new Rotation2d(0)))
            );
        
        cache[TrajectoryType.driveSourceOutsideNotestoCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Next to note to center note Red", .8, .8, false, 
            new Pose2d(3.5, 5.705, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.1436),
                new Translation2d(6.0592, 4.1436)
            ), 
            new Pose2d(7.9, 4.1546, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveCenterNotetoOutsideStageRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Note to podium shot Red", .8, .8, false, 
            new Pose2d(7.9, 4.1546, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.1436),
                new Translation2d(4.8, 4.1436)
            ), 
            new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
            ));

        cache[TrajectoryType.driveOutsideStageLeftCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Podium shot to center left note Red", .8, .8, false,
            new Pose2d(3.7582, 2.7516, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.1436),
                new Translation2d(6.0592, 4.1436)
            ), 
            new Pose2d(7.9, 5.7986, new Rotation2d(0))
            ));
    
        cache[TrajectoryType.driveLeftCenterNotetoOutsideStageRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Left Center note to podium shot Red", .8, .8, false, 
            new Pose2d(7.9, 5.7986, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.1436),
                new Translation2d(4.8, 4.1436)
            ), 
            new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
            ));
        
        cache[TrajectoryType.driveSourceOutsideNotesBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-54)), 
            new Rotation2d(Math.toRadians(0)), 
            calcTrajectory("Source to next to note Blue", .9, .9, false, 
            new Pose2d(0.8, 4.5, new Rotation2d(Math.toRadians(-54))), 
            List.of(), 
            new Pose2d(3.5, 2.5246, new Rotation2d(0)) 
            ));
        
        cache[TrajectoryType.driveSourceOutsideNotestoCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Next to note to center note Blue", .8, .8, false, 
            new Pose2d(3.5, 2.5246, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.086),
                new Translation2d(6.0592, 4.086)
            ), 
            new Pose2d(7.9, 4.075, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveCenterNotetoOutsideStageBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center Note to podium shot Blue", .8, .8, false, 
            new Pose2d(7.9, 4.075, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.086),
                new Translation2d(4.8, 4.086)
            ), 
            new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
            ));

        cache[TrajectoryType.driveOutsideStageLeftCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Podium shot to center left note Blue", .8, .8, false,
            new Pose2d(3.7582, 5.478, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.086),
                new Translation2d(6.0592, 4.086)
            ), 
            new Pose2d(7.9, 2.431, new Rotation2d(0))
            ));
    
        cache[TrajectoryType.driveLeftCenterNotetoOutsideStageBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Left Center note to podium shot Blue", .8, .8, false, 
            new Pose2d(7.9, 2.431, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.086),
                new Translation2d(4.8, 4.086)
            ), 
            new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
            ));

        cache[TrajectoryType.driveFromAmpFarToShootingPosRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Amp Far Note To shooting pos Red", .9, .9, false,
            new Pose2d(7.8, 0.75, new Rotation2d(Math.toRadians(180))), 
            List.of(new Translation2d(5, 1)), 
            new Pose2d(3.5, 2.65, new Rotation2d(Math.toRadians(120)))
            ));
        
        cache[TrajectoryType.driveFromAmpFarToShootingPosBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Amp Far Note To shooting pos Blue", .9, .9, false,
            new Pose2d(7.8, 7.35, new Rotation2d(Math.toRadians(180))), 
            List.of(new Translation2d(5, 7.1)), 
            new Pose2d(3.5, 5.45, new Rotation2d(Math.toRadians(-120)))
            ));

        
        cache[TrajectoryType.driveAmpToFarCenterRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-60)), 
            new Rotation2d(0), 
            calcTrajectory("Amp to Far center note Red", .9, .8, false, 
            new Pose2d(0.8, 1.6296, new Rotation2d(Math.toRadians(-60))), 
            List.of(
                new Translation2d(2.6, 0.42)            // changed from 0.47 to 0.42 to avoid hitting note
            ), 
            new Pose2d(8.4, 0.7696, new Rotation2d(0)))
            );
            
        cache[TrajectoryType.driveFarCenterNoteToPodiumShotRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-10)), 
            calcTrajectory("Far Center note to Podium Shot Red", .9, .8, false, 
            new Pose2d(8.4, 0.7696, new Rotation2d(Math.PI)),
            List.of(), 
            new Pose2d(4.6, 1.9296, new Rotation2d(Math.PI)) 
            ));

       cache[TrajectoryType.drivePodiumShotToNextCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-10)), 
            new Rotation2d(0), 
            calcTrajectory("Podium Shot to Next Center note Red", .9, .8, false, 
            new Pose2d(4.6, 1.9296, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(8.4, 2.4296, new Rotation2d(0)) 
            ));

       cache[TrajectoryType.driveNextCenterNotetoPodiumShotRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-10)), 
            calcTrajectory("Next Center note to Podium Shot Red", .9, .8, false, 
            new Pose2d(8.4, 2.4296, new Rotation2d(Math.PI)),
            List.of(), 
            new Pose2d(4.6, 1.9296, new Rotation2d(Math.PI))
            ));

      cache[TrajectoryType.drivePodiumShotToCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-10)), 
            new Rotation2d(Math.toRadians(20)), 
            calcTrajectory("Podium Shot to Center note Red", .9, .8, false, 
            new Pose2d(4.6, 1.929, new Rotation2d(0)), 
            List.of(new Translation2d(6.647, 2.4946)), 
            new Pose2d(8.2, 4.0996, new Rotation2d(0)) 
            ));

        cache[TrajectoryType.driveCenterNotetoPodiumShotRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(20)), 
            new Rotation2d(0), 
            calcTrajectory("Center note to Podium Shot Red", .9, 8, false, 
            new Pose2d(8.2, 4.0996, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(4.9, 4.0296)), 
            new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI)) 
            ));

        cache[TrajectoryType.driveAmpToFarCenterBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(60)), 
            new Rotation2d(0), 
            calcTrajectory("Amp to Far center note Blue", .9, .8, false, 
            new Pose2d(0.8, 6.6, new Rotation2d(Math.toRadians(60))), 
            List.of(new Translation2d(2.6, 7.8096)),   // changed from 7.7596 to 7.8096
            new Pose2d(8.4, 7.460, new Rotation2d(0)) 
            ));
        
        cache[TrajectoryType.driveFarCenterNoteToPodiumShotBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(10)), 
            calcTrajectory("Center note to Podium Shot Blue", .9, .8, false, 
            new Pose2d(8.4, 7.46, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(4.6, 6.3, new Rotation2d(0)) 
            ));

        cache[TrajectoryType.drivePodiumShotToNextCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(10)), 
            new Rotation2d(0), 
            calcTrajectory("Podium Shot to Next Center note Blue", .9, .8, false, 
            new Pose2d(4.6, 6.3, new Rotation2d(0)), 
            List.of(), 
            new Pose2d(8.4, 5.8, new Rotation2d(0)) 
            ));
        
        cache[TrajectoryType.driveNextCenterNotetoPodiumShotBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(10)), 
            calcTrajectory("Next Center note to Podium Shot Blue", .9, .8, false, 
            new Pose2d(8.4, 5.8, new Rotation2d(0)),
            List.of(), 
            new Pose2d(4.6, 6.3, new Rotation2d(0))
            ));

        cache[TrajectoryType.drivePodiumShotToCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(10)), 
            new Rotation2d(Math.toRadians(-20)), 
            calcTrajectory("Podium Shot to Center note Blue", .9, .8, false, 
            new Pose2d(4.6, 6.3, new Rotation2d(0)), 
            List.of(new Translation2d(6.647, 5.735)), 
            new Pose2d(8.2, 4.13, new Rotation2d(0)) 
            ));
    
        cache[TrajectoryType.driveCenterNotetoPodiumShotBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-20)), 
            new Rotation2d(0), 
            calcTrajectory("Center note to Podium Shot Blue", .9, .8, false, 
            new Pose2d(8.2, 4.13, new Rotation2d(0)), 
            List.of(new Translation2d(4.9, 4.2)), 
            new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI)) 
            ));

        cache[TrajectoryType.driveAmpToSecondFarCenterRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-60)), 
            new Rotation2d(0), 
            calcTrajectory("Amp to Second Far center note Red", .9, .8, false, 
            new Pose2d(0.8, 1.6296, new Rotation2d(Math.toRadians(-60))), 
            List.of(new Translation2d(2.6, 0.47)), 
            new Pose2d(8.4, 2.4296, new Rotation2d(0)))
            );

        cache[TrajectoryType.driveAmpToSecondFarCenterBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(60)), 
            new Rotation2d(0), 
            calcTrajectory("Amp to Second Far center note Blue", .9, .8, false, 
            new Pose2d(0.8, 6.6, new Rotation2d(Math.toRadians(54))), 
            List.of(new Translation2d(2.6, 7.700)), 
            new Pose2d(8.4, 5.8, new Rotation2d(0)) 
            ));
        
        cache[TrajectoryType.driveNextCenterNoteToCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(20)), 
            calcTrajectory("Next center note to middle note red", .9, .25, false, 
            new Pose2d(8.4, 2.4296, new Rotation2d(Math.PI)),
            List.of(new Translation2d(7.2, 3.25)), 
            new Pose2d(8.2, 3.95, new Rotation2d(Math.toRadians(20))) 
            ));

        cache[TrajectoryType.driveNextCenterNoteToCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(Math.toRadians(-20)), 
            calcTrajectory("Next center note to middle note blue", .9, .25, false, 
            new Pose2d(8.4, 5.8, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(7.2, 4.9796)), 
            new Pose2d(8.2, 4.2796, new Rotation2d(Math.toRadians(-20))) 
            ));

      cache[TrajectoryType.driveFirstCenterAmpToNextCenterNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("First center note to next middle note red", .9, .25, false, 
            new Pose2d(8.4, 0.7696, new Rotation2d(Math.PI)),
            List.of(new Translation2d(7.2, 2)), 
            new Pose2d(8.4, 2.4296, new Rotation2d(0)) 
            ));

        cache[TrajectoryType.driveFirstCenterAmpToNextCenterNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("First center note to next middle note blue", .9, .25, false, 
            new Pose2d(8.4, 7.460, new Rotation2d(Math.PI)), 
            List.of(new Translation2d(7.2, 6.2296)), 
            new Pose2d(8.4, 5.8, new Rotation2d(0)) 
            ));

        cache[TrajectoryType.driveFromAmpNoteToSecondCenterRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(-25)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Amp Note To Second Far Note Red", .9, .9, false,
            new Pose2d(2.0, 1.25, new Rotation2d(Math.toRadians(0))), 
            List.of(new Translation2d(5, 1.25)), 
            new Pose2d(7.8, 2.4, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveFromCenterSecondToScorePosRed.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Second Amp Far Note To shooting pos Red", .9, .9, false,
            new Pose2d(7.8, 2.4, new Rotation2d(Math.toRadians(180))), 
            List.of(new Translation2d(5, 1.3)), 
            new Pose2d(3.5, 2.65, new Rotation2d(Math.toRadians(120)))
            ));

        cache[TrajectoryType.driveFromAmpNoteToSecondCenterBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(25)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Center To Second Far Note Blue", .9, .9, false,
            new Pose2d(2.0, 6.85, new Rotation2d(Math.toRadians(0))), 
            List.of(new Translation2d(5, 6.85)), 
            new Pose2d(7.8, 5.7, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveFromCenterSecondToScorePosBlue.value] = new TrajectoryFacing(
            new Rotation2d(Math.toRadians(0)), 
            new Rotation2d(0), 
            calcTrajectory("Drive Second Amp Far Note To shooting pos Blue", .9, .9, false,
            new Pose2d(7.8, 5.7, new Rotation2d(Math.toRadians(180))), 
            List.of(new Translation2d(5, 6.8)), 
            new Pose2d(3.5, 5.45, new Rotation2d(Math.toRadians(-120)))
            ));

        cache[TrajectoryType.driveSourceNextNoteToCenterNoteRightRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Next to note to right center note Red", .8, .8, false, 
            new Pose2d(3.5, 5.705, new Rotation2d(0)),
            List.of(
                new Translation2d(5.5, 6.4296)
            ), 
            new Pose2d(7.9, 5.7986, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveSourceNextNoteToCenterNoteRightBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Next to note to right center note Blue", .8, .8, false, 
            new Pose2d(3.5, 2.5246, new Rotation2d(0)),
            List.of(
                new Translation2d(5.5, 1.8)
            ), 
            new Pose2d(7.9, 2.431, new Rotation2d(0))
            ));
    
        cache[TrajectoryType.drivePodiumShotToCenterRightNoteRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Podium shot to center right note Red", .8, .8, false, 
            new Pose2d(3.7582, 2.7516, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.1436),   
                new Translation2d(6.0592, 4.1436)    
            ), 
            new Pose2d(7.9, 2.431, new Rotation2d(0))
            ));

        cache[TrajectoryType.driveCenterRightNoteToPodiumShotRed.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center right note to Podium Red", .8, .8, false, 
            new Pose2d(7.9, 2.431, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.1436),
                new Translation2d(4.8, 4.1436) 
            ), 
            new Pose2d(3.7582, 2.7516, new Rotation2d(Math.PI))
            ));
            
        cache[TrajectoryType.drivePodiumShotToCenterRightNoteBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Podium shot to center right note Blue", .8, .8, false, 
            new Pose2d(3.7582, 5.478, new Rotation2d(0)),
            List.of(
                new Translation2d(4.8, 4.086),
                new Translation2d(6.0592, 4.086) 

            ), 
            new Pose2d(7.9, 5.7986, new Rotation2d(0))
            ));
                
        cache[TrajectoryType.driveCenterRightNoteToPodiumShotBlue.value] = new TrajectoryFacing(
            new Rotation2d(0), 
            new Rotation2d(0), 
            calcTrajectory("Center right note to Podium Blue", .8, .8, false, 
            new Pose2d(7.9, 5.7986, new Rotation2d(Math.PI)),
            List.of(
                new Translation2d(6.0592, 4.086),
                new Translation2d(4.8, 4.086) 
            ), 
            new Pose2d(3.7582, 5.478, new Rotation2d(Math.PI))
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