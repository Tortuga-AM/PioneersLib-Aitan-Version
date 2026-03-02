package org.team7525.subsystem;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BooleanSupplier;

public abstract class Subsystem<StateType extends SubsystemStates> extends SubsystemBase {

	private Map<StateType, ArrayList<Trigger<StateType>>> triggerMap = new HashMap<>();
	private List<Trigger<StateType>> globalTriggers = new ArrayList<>();
	private List<RunnableTrigger> runnableTriggerList = new ArrayList<>();

	private StateType state = null;
	private Timer stateTimer = new Timer();
	private String subsystemName;

	public Subsystem(String subsystemName, StateType defaultState) {
		if (defaultState == null) {
			throw new RuntimeException("Default state cannot be null!");
		}
		this.subsystemName = subsystemName;
		this.state = defaultState;

		stateTimer.start();
	}

	// State operation
	public void periodic() {
		// Commented out bc bad code
		// Logger.recordOutput(subsystemName + "/state", state.getStateString());
		// if (!DriverStation.isEnabled()) return;
		
		runState();

		checkTriggers();
		checkRunnableTriggers();
	}

	protected abstract void runState();
	
	/**
	 * Called AFTER the subsystem is set to a new state.
	 * Override to implement functionality
	 */
	protected void stateInit() {};

	/**
	 * Called BEFORE the subsystem is set to a new state. 
	 * Override to implement functionality
	 */
	protected void stateExit() {};

	/**
	 * Triggers for state transitions
	 * @param startType The {@link StateType} for starting
	 * @param endType The {@link StateType} for ending
	 * @param condition A {@link BooleanSupplier} that triggers the state transition
	 */
	protected void addTrigger(StateType startType, StateType endType, BooleanSupplier condition) {
		triggerMap.computeIfAbsent(startType, k -> new ArrayList<>())
				.add(new Trigger<>(condition, endType));
	}

	/**
	 * Triggers for state transitions from any state.
	 * This trigger will be checked regardless of the current state.
	 * @param endType The {@link StateType} to transition to
	 * @param condition A {@link BooleanSupplier} that triggers the state transition
	 */
	protected void addTriggerFromAny(StateType endType, BooleanSupplier condition) {
		globalTriggers.add(new Trigger<>(condition, endType));
	}

	protected void addRunnableTrigger(Runnable runnable, BooleanSupplier check) {
		runnableTriggerList.add(new RunnableTrigger(check, runnable));
	}

	private void checkTriggers() {
		List<Trigger<StateType>> triggers = triggerMap.get(state);
		if (triggers != null) {
			for (var trigger : triggers) {
				if (trigger.isTriggered()) {
					setState(trigger.getResultState());
					return;
				}
			}
		}

		for (var trigger : globalTriggers) {
			if (trigger.isTriggered()) {
				setState(trigger.getResultState());
				return;
			}
		}
	}

	private void checkRunnableTriggers() {
		for (var trigger: runnableTriggerList) {
			if (trigger.isTriggered()) {
				trigger.run();
			}
		}
	}

	// Other utilities
	public StateType getState() {
		return state;
	}

	public void setState(StateType state) {
		if (this.state == state) return;

		stateTimer.reset();
		stateExit();

		this.state = state;
		stateInit();

		// Consume any accumulated supplier state (e.g. button pressed while in a different
		// state) so that stale inputs don't immediately trigger a transition out of the new state.
		List<Trigger<StateType>> newStateTriggers = triggerMap.get(state);
		if (newStateTriggers != null) {
			for (var trigger : newStateTriggers) {
				trigger.resetState();
			}
		}
	}

	/**
	 * Gets amount of time the state machine has been in the current state.
	 *
	 * @return time in seconds.
	 */
	protected double getStateTime() {
		return stateTimer.get();
	}

	public Command sysIdDynamic(Direction direction) {
		return new PrintCommand("Please Override Me!");
	}

	public Command sysIdQuasistatic(Direction direction) {
		return new PrintCommand("Please Override Me!");
	}
}
