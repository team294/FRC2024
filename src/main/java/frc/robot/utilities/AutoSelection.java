package frc.robot.utilities;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.CoordType;
import frc.robot.Constants.StopType;
import frc.robot.commands.DriveResetPose;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.Autos.*;
import frc.robot.commands.Sequences.ShootPiece;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.Feeder;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Wrist;


/**
 * Selects the auto routine based upon input from the shuffleboard
 */
public class AutoSelection {

	public static final int NONE = 0;
	public static final int test = 1;
	public static final int shootOne = 2;
	public static final int CenterTwoPieceShoot = 3;
	public static final int SourceTwoPieceShoot = 4;
	public static final int AmpTwoPieceShoot = 5;
	public static final int AmpThreePieceShoot = 6;
	public static final int CenterSourceThreePieceShoot = 7;
	public static final int CenterFourPieceShoot = 8;
	public static final int SourceOnePieceShoot = 9;




	private final AllianceSelection allianceSelection;
	private final TrajectoryCache trajectoryCache;
	private SendableChooser<Integer> autoChooser = new SendableChooser<>();
	
	/**
	 * AutoSelection constructor for command group
	 * Sets up autoPlan widget 
	 */  	
	public AutoSelection(TrajectoryCache trajectoryCache, AllianceSelection allianceSelection, FileLog log) {
		this.trajectoryCache = trajectoryCache;
		this.allianceSelection = allianceSelection;

		// auto selections
		autoChooser.setDefaultOption("None", NONE);
		autoChooser.addOption("CenterTwoPieceShoot", CenterTwoPieceShoot);
		autoChooser.addOption("SourceTwoPieceShoot", SourceTwoPieceShoot);
		// autoChooser.addOption("AmpTwoPieceShoot", AmpTwoPieceShoot);
		// autoChooser.addOption("AmpThreePieceShoot", AmpThreePieceShoot);
		// autoChooser.addOption("OnePieceShoot", shootOne);
		autoChooser.addOption("CenterThreePieceShootAmp", CenterSourceThreePieceShoot);
		autoChooser.addOption("SourceOnePieceShoot", SourceOnePieceShoot);
		autoChooser.addOption("CenterFourPieceNearNoteAuto", CenterFourPieceShoot);
		

	
		// show auto selection widget on Shuffleboard
		SmartDashboard.putData("Autonomous routine", autoChooser);

		// show auto parameters on Shuffleboard
		SmartDashboard.putNumber("Autonomous delay", 0);
		SmartDashboard.putBoolean("Autonomous use vision", false);
	}

	/**
	 * Gets the auto command based upon input from the shuffleboard.
	 * This method is designed to be called at AutonomousInit by Robot.java.
	 * 
	 * @param driveTrain The driveTrain that will be passed to the auto command
	 * @param log        The filelog to write the logs to
	 * @return the command to run
	 */

	public Command getAutoCommand(Intake intake, Wrist wrist, Shooter shooter, Feeder feeder, DriveTrain driveTrain, TrajectoryCache trajectoryCache, BCRRobotState robotState, FileLog log) {
		Command autonomousCommand = null;

		// Get parameters from Shuffleboard
		int autoPlan = autoChooser.getSelected();
		log.writeLogEcho(true, "AutoSelect", "autoPlan",autoPlan);


		double waitTime = SmartDashboard.getNumber("Autonomous delay", 0);
		waitTime = MathUtil.clamp(waitTime, 0, 15);		// make sure autoDelay isn't negative and is only active during auto

		if (autoPlan == NONE) {
			// Starting position = facing drivers
			log.writeLogEcho(true, "AutoSelect", "run None");
			autonomousCommand = new DriveResetPose(180, false, driveTrain, log);
		}

		else if(autoPlan == test){
			log.writeLogEcho(true, "AutoSelect", "run Test");
			autonomousCommand = new DriveTrajectory(CoordType.kRelative, StopType.kCoast, trajectoryCache.cache[TrajectoryCache.TrajectoryType.test.value], driveTrain, log);
		}

		else if(autoPlan == shootOne){
			log.writeLogEcho(true, "AutoSelect", "run One Piece Shoot");
			autonomousCommand = new ShootPiece(true, shooter, feeder, robotState, log);
		}

		else if(autoPlan == CenterTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Center Two Piece Shoot");
			autonomousCommand = new CenterTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source Two Piece Shoot");
			autonomousCommand = new SourceTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AmpTwoPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Amp Two Piece Shoot");
			autonomousCommand = new AmpTwoPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == CenterSourceThreePieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source Center Three Piece Shoot");
			autonomousCommand = new CenterThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == AmpThreePieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Amp Three Piece Shoot");
			autonomousCommand = new AmpThreePieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}
		
		else if(autoPlan == CenterFourPieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Center Four Piece Shoot");
			autonomousCommand = new CenterFourPieceShoot(intake, wrist, shooter, driveTrain, feeder, robotState, trajectoryCache, allianceSelection, log);
		}

		else if(autoPlan == SourceOnePieceShoot){
			log.writeLogEcho(true, "AutoSelect", "run Source One Piece Shoot");
			autonomousCommand = new SourceOneNoteShootLeave(intake, wrist, shooter, driveTrain, feeder, robotState, allianceSelection, log);
		}

        else if (autonomousCommand == null) {
			log.writeLogEcho(true, "AutoSelect", "No autocommand found");
			autonomousCommand = new WaitCommand(1);
		}

		return autonomousCommand;
	}

}