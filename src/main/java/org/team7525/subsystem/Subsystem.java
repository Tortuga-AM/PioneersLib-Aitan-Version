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

	private final Map<StateType, List<Trigger<StateType>>> triggerMap = new HashMap<>();
	private final List<Trigger<StateType>> globalTriggers = new ArrayList<>();
	private final List<RunnableTrigger> runnableTriggerList = new ArrayList<>();

	private final Map<StateType, List<Runnable>> stateEntryActions = new HashMap<>();
	private final Map<StateType, List<Runnable>> stateExitActions = new HashMap<>();
	private final Map<StateType, Map<StateType, List<Runnable>>> transitionCallbacks = new HashMap<>();

	private StateType state = null;
	private final Timer stateTimer = new Timer();
	private final String subsystemName;

	public Subsystem(String subsystemName, StateType defaultState) {
		if (defaultState == null) {
			throw new RuntimeException("Default state cannot be null!");
		}
		this.subsystemName = subsystemName;
		this.state = defaultState;

		stateTimer.start();
	}

	// ── Periodic loop ─────────────────────────────────────────────────────────

	public void periodic() {
		SmartDashboard.putString(subsystemName + "/State", state.getStateString());

		runState();

		checkTriggers();
		checkRunnableTriggers();
	}

	protected abstract void runState();

	/**
	 * Called AFTER the subsystem transitions into a new state.
	 * Override to implement functionality that should happen on every state entry.
	 *
	 * <p>Execution order within a state transition:
	 * <ol>
	 *   <li>{@code stateInit()} (this override)</li>
	 *   <li>Per-state entry actions registered via {@link #addStateEntryAction(SubsystemStates, Runnable)}</li>
	 * </ol>
	 * Prefer {@link #addStateEntryAction(SubsystemStates, Runnable)} for per-state
	 * logic to avoid large switch statements.
	 */
	protected void stateInit() {}

	/**
	 * Called BEFORE the subsystem transitions out of the current state.
	 * Override to implement functionality that should happen on every state exit.
	 *
	 * <p>Execution order within a state transition:
	 * <ol>
	 *   <li>{@code stateExit()} (this override)</li>
	 *   <li>Per-state exit actions registered via {@link #addStateExitAction(SubsystemStates, Runnable)}</li>
	 * </ol>
	 * Prefer {@link #addStateExitAction(SubsystemStates, Runnable)} for per-state
	 * logic to avoid large switch statements.
	 */
	protected void stateExit() {}

	// ── Trigger registration ──────────────────────────────────────────────────

	/**
	 * Registers a state transition from {@code startType} to {@code endType}
	 * whenever {@code condition} returns {@code true}.
	 *
	 * @param startType the state this trigger is active in
	 * @param endType   the state to transition to when the condition fires
	 * @param condition the condition that triggers the transition
	 */
	protected void addTrigger(StateType startType, StateType endType, BooleanSupplier condition) {
		triggerMap.computeIfAbsent(startType, k -> new ArrayList<>())
				.add(new Trigger<>(condition, endType));
	}

	/**
	 * Registers a state transition to {@code endType} from <em>any</em> state whenever
	 * {@code condition} returns {@code true}.
	 *
	 * <p>State-specific triggers are checked first; global triggers are only evaluated
	 * when no state-specific trigger fired.
	 *
	 * @param endType   the state to transition to
	 * @param condition the condition that triggers the transition
	 */
	protected void addTriggerFromAny(StateType endType, BooleanSupplier condition) {
		globalTriggers.add(new Trigger<>(condition, endType));
	}

	/**
	 * Registers an automatic transition from {@code fromState} to {@code toState} after
	 * spending at least {@code seconds} seconds in {@code fromState}.
	 *
	 * @param fromState the state to auto-transition from
	 * @param toState   the state to transition to
	 * @param seconds   the minimum time (in seconds) to spend in {@code fromState}
	 */
	protected void addTimedTrigger(StateType fromState, StateType toState, double seconds) {
		addTrigger(fromState, toState, () -> getStateTime() >= seconds);
	}

	/**
	 * Registers a {@link Runnable} that is executed every loop while {@code check}
	 * returns {@code true}, without causing a state transition.
	 *
	 * @param runnable the action to execute
	 * @param check    the condition that gates execution
	 */
	protected void addRunnableTrigger(Runnable runnable, BooleanSupplier check) {
		runnableTriggerList.add(new RunnableTrigger(check, runnable));
	}

	// ── Lifecycle action registration ─────────────────────────────────────────

	/**
	 * Registers an action to run every time the subsystem enters {@code state}.
	 * Multiple actions can be registered for the same state; they run in registration order,
	 * <em>after</em> the {@link #stateInit()} override returns.
	 *
	 * <p>This is the preferred alternative to overriding {@link #stateInit()} with a
	 * switch statement.
	 *
	 * @param state  the state whose entry triggers the action
	 * @param action the action to run on entry
	 */
	protected void addStateEntryAction(StateType state, Runnable action) {
		stateEntryActions.computeIfAbsent(state, k -> new ArrayList<>()).add(action);
	}

	/**
	 * Registers an action to run every time the subsystem exits {@code state}.
	 * Multiple actions can be registered for the same state; they run in registration order,
	 * <em>after</em> the {@link #stateExit()} override returns.
	 *
	 * <p>This is the preferred alternative to overriding {@link #stateExit()} with a
	 * switch statement.
	 *
	 * @param state  the state whose exit triggers the action
	 * @param action the action to run on exit
	 */
	protected void addStateExitAction(StateType state, Runnable action) {
		stateExitActions.computeIfAbsent(state, k -> new ArrayList<>()).add(action);
	}

	/**
	 * Registers a callback to run whenever the subsystem transitions specifically from
	 * {@code fromState} to {@code toState}.
	 *
	 * @param fromState the source state
	 * @param toState   the destination state
	 * @param action    the callback to run on this specific transition
	 */
	protected void addTransitionCallback(StateType fromState, StateType toState, Runnable action) {
		transitionCallbacks
				.computeIfAbsent(fromState, k -> new HashMap<>())
				.computeIfAbsent(toState, k -> new ArrayList<>())
				.add(action);
	}

	// ── Fluent transition builder ─────────────────────────────────────────────

	/**
	 * Returns a {@link TransitionBuilder} for fluently registering transitions
	 * out of {@code fromState}.
	 *
	 * <pre>{@code
	 * fromState(States.IDLE)
	 *     .goTo(States.INTAKING, driver.a()::getAsBoolean)
	 *     .goTo(States.SCORING,  () -> driver.getRightTriggerAxis() > 0.5)
	 *     .afterSeconds(States.IDLE, 30.0); // safety timeout back to IDLE
	 * }</pre>
	 *
	 * @param fromState the state to register outgoing transitions from
	 * @return a builder for chaining transition registrations
	 */
	protected TransitionBuilder fromState(StateType fromState) {
		return new TransitionBuilder(fromState);
	}

	/**
	 * Fluent builder returned by {@link #fromState(SubsystemStates)}.
	 * All calls immediately register the corresponding trigger in the enclosing subsystem.
	 */
	public final class TransitionBuilder {
		private final StateType builderFromState;

		private TransitionBuilder(StateType fromState) {
			this.builderFromState = fromState;
		}

		/**
		 * Registers a transition to {@code toState} when {@code condition} is {@code true}.
		 *
		 * @param toState   the destination state
		 * @param condition the BooleanSupplier condition
		 * @return {@code this} for chaining
		 */
		public TransitionBuilder goTo(StateType toState, BooleanSupplier condition) {
			addTrigger(builderFromState, toState, condition);
			return this;
		}

		/**
		 * Registers an automatic transition to {@code toState} after spending
		 * {@code seconds} seconds in the source state.
		 *
		 * @param toState the destination state
		 * @param seconds the time in seconds to wait before transitioning
		 * @return {@code this} for chaining
		 */
		public TransitionBuilder afterSeconds(StateType toState, double seconds) {
			addTimedTrigger(builderFromState, toState, seconds);
			return this;
		}
	}

	// ── State machine internals ───────────────────────────────────────────────

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
		for (var trigger : runnableTriggerList) {
			if (trigger.isTriggered()) {
				trigger.run();
			}
		}
	}

	// ── Public API ────────────────────────────────────────────────────────────

	/** Returns the current state. */
	public StateType getState() {
		return state;
	}

	/** Returns {@code true} if the subsystem is currently in {@code queryState}. */
	public boolean isInState(StateType queryState) {
		return this.state == queryState;
	}

	public void setState(StateType newState) {
		if (this.state == newState) return;

		StateType previousState = this.state;

		stateTimer.reset();

		stateExit();

		// Per-state exit actions registered via addStateExitAction (run after the override)
		List<Runnable> exitActions = stateExitActions.get(previousState);
		if (exitActions != null) exitActions.forEach(Runnable::run);

		// Per-transition callbacks registered via addTransitionCallback
		Map<StateType, List<Runnable>> outgoing = transitionCallbacks.get(previousState);
		if (outgoing != null) {
			List<Runnable> callbacks = outgoing.get(newState);
			if (callbacks != null) callbacks.forEach(Runnable::run);
		}

		this.state = newState;

		stateInit();

		// Per-state entry actions registered via addStateEntryAction
		List<Runnable> entryActions = stateEntryActions.get(newState);
		if (entryActions != null) entryActions.forEach(Runnable::run);

		// Consume stale supplier state for the new state's triggers AND global triggers.
		// This prevents button presses accumulated while in a different state from
		// immediately firing a transition out of the newly-entered state.
		List<Trigger<StateType>> newStateTriggers = triggerMap.get(newState);
		if (newStateTriggers != null) {
			for (var trigger : newStateTriggers) {
				trigger.resetState();
			}
		}
		for (var trigger : globalTriggers) {
			trigger.resetState();
		}
	}

	/**
	 * Returns how long the subsystem has been in the current state.
	 *
	 * @return elapsed time in seconds
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
