// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.util.concurrent.Event;

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

    //True if in far shot mode, false to revert to speaker mode boolean
    // private boolean farShotMode = false;
    public static enum ShotMode {
        SPEAKER,
        AMP,
        SHORT_PASS,
        FAR_PASS,
        VISION_FAR_PASS,
        VISION_MID_PASS
    }

    private ShotMode shotMode = ShotMode.SPEAKER;

    public static enum EventType {
        COMPETITION,
        OPEN_HOUSE,
        OUTREACH
    }

    private EventType eventType = EventType.COMPETITION;

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
     * Sets the current State of the robot
     * @param state new State
     */
    public void setState(State state) {
        this.state = state;
    }

    /**
     * returns if shooter is in mode for a Far shot (lobbing note towards alliance partner)
     * @return true = far shot, false = normal speaker shot
     */
    public ShotMode getShotMode(){
        return shotMode;
    }

    /**
     * Set if in mode for a Far shot (lobbing note towards alliance partner)
     * @param farShotMode true = far shot, false = normal speaker shot
     */
    public void setShotMode(ShotMode shotMode){
        this.shotMode = shotMode;
    }

    /**
     * Get the current event type of the robot
     * @return event type
     */
    public EventType getEventType(){
        return eventType;
    }

    /**
     * Set the event type the robot is in
     * @param eventType new event type
     */
    public void setEventType(EventType eventType){
        this.eventType = eventType;
    }
}