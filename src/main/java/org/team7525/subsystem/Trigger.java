package org.team7525.subsystem;

import java.util.function.BooleanSupplier;

public class Trigger<StateType extends SubsystemStates> {
    BooleanSupplier supplier;
    StateType state;

    public Trigger(BooleanSupplier supplier, StateType state) {
        this.supplier = supplier;
        this.state = state;
    }

    public boolean isTriggered() {
        return supplier.getAsBoolean();
    }

    public StateType getResultState() {
        return state;
    }

    /**
     * Consumes the current supplier value to clear any accumulated state.
     *
     * <p>WPILib button suppliers such as {@code GenericHID.getRawButtonPressed()} accumulate
     * "pressed" state between calls — meaning they return {@code true} not just for the loop in
     * which the button was pressed, but for every subsequent call until they are read. If a button
     * is pressed while the subsystem is in a state that has no trigger for it, the press is never
     * consumed. Later, if the subsystem transitions into a state that <em>does</em> have a trigger
     * for that button, the stale press would fire the trigger immediately.
     *
     * <p>Calling this method when entering the new state discards any such accumulated value,
     * ensuring triggers only fire in response to button events that occur while already in that state.
     */
    public void resetState() {
        supplier.getAsBoolean();
    }
}
