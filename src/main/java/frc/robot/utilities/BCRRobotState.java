// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

/** A wrapper class for State objects (see state machine file) */
public class BCRRobotState {
    /** The enum that keeps track of all possible states for the robot */
    public static enum State {
        IDLE,
        INTAKING,
        SHOOTING
    }

    // The current State of the robot
    private State state;

    //True if in speaker mode, false if in amp mode
    private boolean speakerMode = true; 

    /** Creates a new BCRRobotState with the default State value of IDLE */
    public BCRRobotState() {
        setState(State.IDLE);
    }

    /**
     * Creates a new BCRRobotState with the given initial State
     * @param state initial State
     */
    public BCRRobotState(State state) {
        setState(state);
    }

    /** Gets the current State of the robot */
    public State getState() {
        return state;
    }

    /**
     * returns if shooter/wrist are in speaker mode or amp mode
     * @return true = speaker mode, false = amp mode
     */
    public boolean isSpeakerMode(){
        return speakerMode;
    }

    /**
     * Set if the robot is in speaker mode or amp mode
     * @param speakerMode true = speaker mode, false = amp mode
     */
    public void setSpeakerMode(boolean speakerMode){
        this.speakerMode = speakerMode;
    }

    /**
     * Sets the current State of the robot
     * @param state new State
     */
    public void setState(State state) {
        this.state = state;
    }
}