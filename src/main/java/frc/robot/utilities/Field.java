// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.FieldConstants;

/**
 * A class representing field coordinates.
 * <p> Field coordinates include:
 * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
 * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
 * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
 */
public class Field {
    //Robot probably 31" with bumpers
    private final Rotation2d oneEighty = new Rotation2d(Math.PI);
    private final Rotation2d zero = new Rotation2d(0.0);
    private final Rotation2d ninety = new Rotation2d(Math.PI/2);
    private final Rotation2d twoSeventy = new Rotation2d(Math.PI*1.5);
    private final Rotation2d oneTwenty = new Rotation2d(Math.PI*(2/3));
    private final Rotation2d twoForty = new Rotation2d(Math.PI*(4/3));
    private final Rotation2d sixty = new Rotation2d(Math.PI/3);
    private final Rotation2d threeHundred = new Rotation2d(Math.PI*(5/3));

    //Community -> Loading
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] BlueCommunityColumnInitial = {
        new Pose2d(2.03, 0.512826, oneEighty), //54.25+(31/2)*sqrt(2)+6 inches to meters for x value
        new Pose2d(2.03, 1.071626, oneEighty), 
        new Pose2d(2.03, 1.630426, oneEighty), 
        new Pose2d(2.03, 2.189226, oneEighty), 
        new Pose2d(2.03, 2.748026, oneEighty), 
        new Pose2d(2.03, 3.306826, oneEighty), 
        new Pose2d(2.03, 3.865626, oneEighty), 
        new Pose2d(2.03, 4.424426, oneEighty), 
        new Pose2d(2.03, 4.983226, oneEighty) 
    };

    //Community -> Place part
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] BlueCommunityColumnFinal = {
        new Pose2d(1.77165, 0.512826, oneEighty), //54.25+(31/2) inches to meters for x value
        new Pose2d(1.77165, 1.071626, oneEighty), 
        new Pose2d(1.77165, 1.630426, oneEighty), 
        new Pose2d(1.77165, 2.189226, oneEighty), 
        new Pose2d(1.77165, 2.748026, oneEighty), 
        new Pose2d(1.77165, 3.306826, oneEighty), 
        new Pose2d(1.77165, 3.865626, oneEighty), 
        new Pose2d(1.77165, 4.424426 - 0.100, oneEighty),           // was 4.424426.  F2:  In LA elims, cube was landing on border of position 8 and 9, so move 4in (0.1m) towards pos 7.
        new Pose2d(1.77165, 4.983226, oneEighty) 
    };

    //Loading -> Community
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] RedCommunityColumnInitial = {
        new Pose2d(2.03, 3.020568, oneEighty), //118.92 inches
        new Pose2d(2.03, 3.579368, oneEighty), //140.92 inches
        new Pose2d(2.03, 4.138168, oneEighty), //162.92 inches
        new Pose2d(2.03, 4.696968, oneEighty), //184.92 inches
        new Pose2d(2.03, 5.255768, oneEighty), //206.92 inches
        new Pose2d(2.03, 5.814568, oneEighty), //228.92 inches
        new Pose2d(2.03, 6.373368, oneEighty), //250.92 inches
        new Pose2d(2.03, 6.932168, oneEighty), //272.92 inches
        new Pose2d(2.03, 7.490968, oneEighty)  //294.92 inches
    };

    //Community -> Place part
    // #0 = furthest to right (from driver point of view)
    // #8 = furthest to left (from driver point of view)
    private final Pose2d[] RedCommunityColumnFinal = {
        new Pose2d(1.77165, 3.020568, oneEighty), 
        new Pose2d(1.77165, 3.579368 + 0.100, oneEighty),           // was 3.579368.  F1:  In LA elims, cube was landing on border of position 1 and 2, so move 4in (0.1m) towards pos 3.
        new Pose2d(1.77165, 4.138168, oneEighty), 
        new Pose2d(1.77165, 4.696968, oneEighty), 
        new Pose2d(1.77165, 5.255768, oneEighty), 
        new Pose2d(1.77165, 5.814568, oneEighty), 
        new Pose2d(1.77165, 6.373368, oneEighty), 
        new Pose2d(1.77165, 6.932168, oneEighty), 
        new Pose2d(1.77165, 7.490968, oneEighty) 
    };

    // #0 -> 2 = right to left, closest to driver (from driver point of view)
    // #3 -> 5 = right to left, furthest from driver (from driver point of view)
    //  5  4  3
    //  Station
    //  2  1  0
    // Community
    private final Pose2d[] BlueStationInitial = {
        new Pose2d(2.148713, 2.130489, oneEighty),
        new Pose2d(2.148713, 2.748026, oneEighty),
        new Pose2d(2.148713, 3.365563, oneEighty),
        new Pose2d(4.855718, 2.130489, oneEighty),
        new Pose2d(4.855718, 2.748026, oneEighty),
        new Pose2d(4.855718, 3.365563, oneEighty)
    };
    
    //Community/Field -> Station
    // #0 = furthest to right (from driver point of view)
    // #2 = furthest to left (from driver point of view)
    private final Pose2d[] BlueStationFinal = {
        new Pose2d(3.8354, 2.130489, oneEighty), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 2.748026, oneEighty),
        new Pose2d(3.8354, 3.365563, oneEighty)
    };

    // #0 -> 2 = right to left, closest to driver (from driver point of view)
    // #3 -> 5 = right to left, furthest from driver (from driver point of view)
    //  5  4  3
    //  Station
    //  2  1  0
    // Community
    private final Pose2d[] RedStationInitial = {
        new Pose2d(2.148713, 4.646867, oneEighty),
        new Pose2d(2.148713, 5.264404, oneEighty),
        new Pose2d(2.148713, 5.881941, oneEighty),
        new Pose2d(4.855718, 4.646867, oneEighty),
        new Pose2d(4.855718, 5.264404, oneEighty),
        new Pose2d(4.855718, 5.881941, oneEighty)
    };
    
    //Community/Field -> Station
    // #0 = furthest to right (from driver point of view)
    // #2 = furthest to left (from driver point of view)
    private final Pose2d[] RedStationFinal = {
        new Pose2d(3.8354, 4.646867, oneEighty), //Always faces away from communities, this could cause issues
        new Pose2d(3.8354, 5.264404, oneEighty), //Values found by adding loading zone width (99.07 inches) to Blue values
        new Pose2d(3.8354, 5.881941, oneEighty)
    };

    // BLUE  
    //IDs 1 and 2 are loading zone
    //ID 3 is amp
    //IDs 4 and 5 are speaker
    //IDs 6 - 8 are stage
    
    // RED
    //IDs 9 and 10 are loading zone
    //ID 11 is amp
    //IDs 12 and 13 are speaker
    //IDs 14 - 16 are stage
    private final AprilTag[] AprilTags = {
        new AprilTag(1, new Pose3d(new Pose2d(15.0794638570895, 0.245871867229192, oneTwenty))),      // 120 degrees
        new AprilTag(2, new Pose3d(new Pose2d(16.1851252600324, 0.883665522820618, oneTwenty))),      // 120 degrees
        new AprilTag(3, new Pose3d(new Pose2d(1.84149900559054, 8.20419556973439, twoSeventy))),      // 270 degrees
        new AprilTag(4, new Pose3d(new Pose2d(-0.0380999794260111, 5.5478650041529, zero))),            // 0 degrees
        new AprilTag(5, new Pose3d(new Pose2d(-0.0380999794260111, 4.98271530933373, zero))),           // 0 degrees
        new AprilTag(6, new Pose3d(new Pose2d(5.32078912677387, 4.10514578322128, zero))),            // 0 degrees
        new AprilTag(7, new Pose3d(new Pose2d(4.64133949367667, 4.49833757089771, oneTwenty))),       // 120 degrees
        new AprilTag(8, new Pose3d(new Pose2d(4.64133949367667, 3.71322399485904, twoForty))),        // 240 degrees
        new AprilTag(9, new Pose3d(new Pose2d(0.356107807701784, 0.883665522820618, sixty))),         // 60 degrees
        new AprilTag(10, new Pose3d(new Pose2d(1.46151521078179, 0.245871867229192, sixty))),         // 60 degrees
        new AprilTag(11, new Pose3d(new Pose2d(14.700750061595, 8.20419556973439, twoSeventy))),      // 270 degrees
        new AprilTag(12, new Pose3d(new Pose2d(16.5793330471602, 4.98271530933373, oneEighty))),      // 180 degrees
        new AprilTag(13, new Pose3d(new Pose2d(16.5793330471602, 5.5478650041529, oneEighty))),       // 180 degrees
        new AprilTag(14, new Pose3d(new Pose2d(11.9047195714514, 3.71322399485904, threeHundred))),   // 300 degrees
        new AprilTag(15, new Pose3d(new Pose2d(11.9047195714514, 4.49833757089771, sixty))),          // 60 degrees
        new AprilTag(16, new Pose3d(new Pose2d(11.2201899410974, 4.10514578322128, oneEighty)))       // 180 degrees
    };

    private final Pose2d BlueLoadingStationInitial = new Pose2d(14.37878, 6.499796, zero);  // 16.17878-1.7, 6.749796-0.25
    private final Pose2d BlueLoadingStationFinal = new Pose2d(14.57878, 6.499796, zero);    // 16.17878-1.6, 6.749796-0.25
    private final Pose2d RedLoadingStationInitial = new Pose2d(14.37878, 1.003998, zero);   // 16.17878-1.7, 1.253998-0.25
    private final Pose2d RedLoadingStationFinal = new Pose2d(14.57878, 1.003998, zero);     // 16.17878-1.6, 1.253998-0.25
    

    private final AllianceSelection alliance;
    private final FileLog log;

    /**
     * Create a field object that can provide various field locations.  All field
     * locations are Pose2d objects based on the current alliance that is selected.
     * Pose components include:
     * <p> Robot X location in the field, in meters (0 = field edge in front of driver station, + = away from our drivestation)
     * <p> Robot Y location in the field, in meters (0 = right edge of field when standing in driver station, + = left when looking from our drivestation)
     * <p> Robot angle on the field (0 = facing away from our drivestation, + to the left, - to the right)
     * @param alliance Alliance object to provide the currently selected alliance
     */
    public Field(AllianceSelection alliance, FileLog log){
        this.alliance = alliance;
        this.log = log;
    }

    /**
	 * Gets the initial column position (in front of scoring position, but backed up with a little room for the robot to rotate).
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getInitialColumn(int column) throws IndexOutOfBoundsException {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnInitial[column-1];
            }
            else {
                return RedCommunityColumnInitial[column-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Initial Column %d out of range", column));
        }
    }

    /**
	 * Gets the column scoring position.
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
	 * 
	 * @param column The column that will be returned (1-9)
	 */
    public Pose2d getFinalColumn(int column) throws IndexOutOfBoundsException {
        if(column < 10 && column > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueCommunityColumnFinal[column-1];
            }
            else {
                return RedCommunityColumnFinal[column-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Column ID %d out of range", column));
        }
    }

    /**
	 * Gets the position to approach the station from
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position 1-3 Lowest-Highest Communtiy Side | 4-6 Lowest-Highest Field Side
	 */
    public Pose2d getStationInitial(int position) throws IndexOutOfBoundsException {
        if(position < 7 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationInitial[position-1];
            }
            else {
                return RedStationInitial[position-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Station Position %d out of range", position));
        }
    }

    /**
	 * Gets the center positions on the station
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position 1-3 Lowest-Hightest (Y-Axis)
	 */
    public Pose2d getStationCenter(int position) throws IndexOutOfBoundsException {
        if(position < 4 && position > 0){
            if(alliance.getAlliance() == Alliance.Blue) {
                return BlueStationFinal[position-1];
            }
            else {
                return RedStationFinal[position-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("Station Center position %d out of range", position));
        }
    }

    /**
	 * Gets the position of a specified April Tag (1-8)
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
	 * 
	 * @param position
	 */
    public AprilTag getAprilTag(int ID) throws IndexOutOfBoundsException {
        if(ID < 9 && ID > 0) {
            if(alliance.getAlliance() == Alliance.Blue) {
                return AprilTags[ID-1];
            } else {
                return AprilTags[ID-1];
            }
        } else {
            throw new IndexOutOfBoundsException(String.format("AprilTag ID %d out of range", ID));
        }
    }

    /**
	 * Gets the position to approach the loading station from
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * @return Loading station initial position
     */
    public Pose2d getLoadingPositionInitial() {
        if(alliance.getAlliance() == Alliance.Blue){
            return BlueLoadingStationInitial;
        } else {
            return RedLoadingStationInitial;
        }
    }

    /**
	 * Gets the position to grab a piece at the loading station
     * <p> Note that the position will be different for red vs blue alliance, based on the current alliance in the alliance object.
     * @return Loading station "get piece" position
     */
    public Pose2d getLoadingPositionFinal() {
        if(alliance.getAlliance() == Alliance.Blue){
            return BlueLoadingStationFinal;
        } else {
            return RedLoadingStationFinal;
        }
    }

    /**
     * gets the game's AprilTagFieldLayout for vision
     * @return AprilTagFieldLayout for vision
     */
    public AprilTagFieldLayout getAprilTagFieldLayout() {
        List<AprilTag> atList;
        
        if(alliance.getAlliance() == Alliance.Blue) {
            log.writeLogEcho(true, "Field", "getAprilTagFieldLayout", "Loaded", "blue");
            atList = Arrays.asList(AprilTags);
        } else {
            log.writeLogEcho(true, "Field", "getAprilTagFieldLayout", "Loaded", "red");
            atList = Arrays.asList(AprilTags);
        }

        return new AprilTagFieldLayout(atList, FieldConstants.length, FieldConstants.width);
    }

    /**
     * gets alliance
     * @return Alliance current selected alliance color
     */
    public Alliance getAlliance() {
        return alliance.getAlliance();
    }

    /**
     * Returns the column of the closest goal (1-9), based on the current game piece setting in the manipulator.
     * <p> #1 = furthest to right (from driver point of view)
     * <p> #9 = furthest to left (from driver point of view)
     * @return Column of the closest goal (1-9) for the current game piece setting in the manipulator
     */
    public int getClosestGoal(Pose2d robotPose, boolean isCone) {
        int closestGoal;
        double robotY = robotPose.getY();

        if(alliance.getAlliance() == Alliance.Blue){//Alliance Blue
            closestGoal = 0;
            if(isCone){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 1 || i == 4 || i == 7){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - BlueCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            } else {
                closestGoal = 1;
                for(int i = 1; i < 9; i++){
                    if(i != 1 && i != 4 && i != 7){
                        continue;
                    }
                    if(Math.abs(robotY - BlueCommunityColumnFinal[i].getY()) < Math.abs(robotY - BlueCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            }
            log.writeLogEcho(true, "Field", "GetClosetGoal", "Alliance", "Blue", "Cone", isCone, 
                "Column", closestGoal+1, "X", BlueCommunityColumnFinal[closestGoal].getX(),
                "Y", BlueCommunityColumnFinal[closestGoal].getY(), "Rot", BlueCommunityColumnFinal[closestGoal].getRotation().getDegrees());
        } else {
            closestGoal = 0;
            if(isCone){//Carrying Cone
                for(int i = 0; i < 9; i++){
                    if(i == 1 || i == 4 || i == 7){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - RedCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            } else {
                closestGoal = 1;
                for(int i = 1; i < 9; i++){
                    if(i != 1 && i != 4 && i != 7){
                        continue;
                    }
                    if(Math.abs(robotY - RedCommunityColumnFinal[i].getY()) < Math.abs(robotY - RedCommunityColumnFinal[closestGoal].getY())){
                        closestGoal = i;
                    }
                }
            }
            log.writeLogEcho(true, "Field", "GetClosetGoal", "Alliance", "Red", "Cone", isCone, 
                "Column", closestGoal+1, "X", RedCommunityColumnFinal[closestGoal].getX(),
                "Y", RedCommunityColumnFinal[closestGoal].getY(), "Rot", RedCommunityColumnFinal[closestGoal].getRotation().getDegrees());
        }
        closestGoal++;          // Adjust for 0-based index in array
        SmartDashboard.putNumber("Closest Goal", closestGoal);
        return closestGoal;
    }
}
