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

    //True if in mode to score in speaker, false if in mode to score in amp
    private boolean speakerMode = true; 

    //True if in far shot mode, false to revert to speaker mode boolean
    private boolean farShotMode = false;

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
     * returns if shooter/wrist are in mode to score in speaker or in mode to score in amp
     * @return true = speaker mode, false = amp mode
     */
    public boolean isSpeakerMode(){
        return speakerMode;
    }
    
    /**
     * returns if shooter is in mode for a Far shot (lobbing note towards alliance partner)
     * @return true = far shot, false = normal speaker shot
     */
    public boolean isFarShotMode(){
        return farShotMode;
    }

    /**
     * Set if in mode to score in speaker or in mode to score in amp
     * @param speakerMode true = speaker mode, false = amp mode
     */
    public void setSpeakerMode(boolean speakerMode){
        this.speakerMode = speakerMode;
    }

    /**
     * Set if in mode for a Far shot (lobbing note towards alliance partner)
     * @param farShotMode true = far shot, false = normal speaker shot
     */
    public void setFarShotMode(boolean farShotMode){
        this.farShotMode = farShotMode;
    }

    /**
     * Sets the current State of the robot
     * @param state new State
     */
    public void setState(State state) {
        this.state = state;
    }
}