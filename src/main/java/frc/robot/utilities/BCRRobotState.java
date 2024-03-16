// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.BCRColor;
import frc.robot.Constants.LEDConstants.LEDSegmentRange;
import frc.robot.subsystems.LED;

/** A wrapper class for State objects (see state machine file) */
public class BCRRobotState {
    /** The enum that keeps track of all possible states for the robot */
    public static enum State {
        IDLE_NO_PIECE,           // Driving around with no piece
        IDLE_WITH_PIECE,         // Have piece, not shooting
        INTAKING,                // Intake running
        SHOOTING,                // Robot is in the process of shooting a piece
        STICKY_FAULTS            // Errors
    }

    // The current State of the robot
    private State state;
    // The LED subsystem
    private LED led;

    /** Creates a new BCRRobotState with the default State value of IDLE_NO_PIECE
     * @param led the LED subsystem to match color with state
    */
    public BCRRobotState(LED led) {
        this.led = led;
        setState(State.IDLE_NO_PIECE);
    }

    /**
     * Creates a new BCRRobotState with the given initial State
     * @param state initial State
     * @param led the LED subsystem to match color with state
     */
    public BCRRobotState(State state, LED led) {
        this.led = led;
        setState(state);
    }

    /** Gets the current State of the robot */
    public State getState() {
        return state;
    }

    /**
     * Sets the current State of the robot
     * @param state new State
     */
    public void setState(State state) {
        this.state = state;
        led.updateStateAll(this.state);
    }
}