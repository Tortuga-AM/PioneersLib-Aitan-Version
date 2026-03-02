package org.team7525.subsystem.behaviortree;

import java.util.Arrays;
import java.util.function.BooleanSupplier;

/**
 * Static factory methods for building behavior tree nodes.
 *
 * <p>A <em>behavior tree</em> is an alternative to the explicit state-machine trigger
 * map. Instead of enumerating every A→B transition, you declare behaviors in
 * <em>priority order</em> inside a {@link #selector}. The tree is re-evaluated from
 * the root on every robot loop cycle, so high-priority branches preempt lower-priority
 * ones the moment their conditions become true.
 *
 * <h2>State machine vs. behavior tree — when to use each</h2>
 * <table>
 *   <tr><th></th><th>State machine ({@code addTrigger})</th><th>Behavior tree ({@code setBehaviorTree})</th></tr>
 *   <tr><td><b>Best for</b></td>
 *       <td>Simple subsystems with clear, discrete hardware setpoints and well-defined
 *           one-way transitions (e.g. pivot: IDLE→SCORE, SCORE→IDLE).</td>
 *       <td>Subsystems with many overlapping conditions, priority-based preemption
 *           (e.g. e-stop from <em>any</em> state), or complex conditional logic.</td></tr>
 *   <tr><td><b>Readability</b></td>
 *       <td>Explicit: you see exactly which state transitions to which.</td>
 *       <td>Declarative: priorities are visible at a glance from top to bottom.</td></tr>
 *   <tr><td><b>Global transitions</b></td>
 *       <td>Requires {@code addTriggerFromAny()} to reach a state from everywhere.</td>
 *       <td>Free: put it as the first child of the root selector.</td></tr>
 *   <tr><td><b>Transition count</b></td>
 *       <td>Grows as O(states²) in the worst case.</td>
 *       <td>O(states) — one priority rule per state.</td></tr>
 * </table>
 *
 * <h2>Example — intake subsystem</h2>
 * <pre>{@code
 * // State machine (7 explicit transitions)
 * addTrigger(STOWED,   INTAKING, leftBumper::getAsBoolean);
 * addTrigger(INTAKING, STOWED,   () -> !leftBumper.getAsBoolean());
 * addTrigger(INTAKING, HOLDING,  () -> !beamBreak.get());
 * addTrigger(HOLDING,  EJECTING, () -> leftTrigger > 0.5);
 * addTrigger(HOLDING,  STOWED,   rightBumper::getAsBoolean);
 * addTrigger(EJECTING, STOWED,   () -> leftTrigger <= 0.5);
 * addTriggerFromAny(ESTOP, backButton::getAsBoolean);
 *
 * // Behavior tree (5 priority rules — same logic, more readable)
 * setBehaviorTree(
 *     BT.selector(
 *         BT.sequence(BT.condition(backButton::getAsBoolean),       go(ESTOP)),
 *         BT.sequence(BT.condition(() -> leftTrigger > 0.5),        go(EJECTING)),
 *         BT.sequence(BT.condition(() -> !beamBreak.get()),         go(HOLDING)),
 *         BT.sequence(BT.condition(leftBumper::getAsBoolean),       go(INTAKING)),
 *         go(STOWED)
 *     )
 * );
 * }</pre>
 *
 * @see org.team7525.subsystem.Subsystem#setBehaviorTree(BehaviorNode)
 * @see org.team7525.subsystem.Subsystem#go(SubsystemStates)
 */
public final class BT {

	private BT() {}

	/**
	 * Creates a {@link ConditionNode} that returns {@link NodeStatus#SUCCESS}
	 * when {@code condition} is {@code true}, {@link NodeStatus#FAILURE} otherwise.
	 *
	 * @param condition the boolean condition to evaluate each tick
	 * @return a condition leaf node
	 */
	public static BehaviorNode condition(BooleanSupplier condition) {
		return new ConditionNode(condition);
	}

	/**
	 * Creates a {@link SequenceNode} that ticks children left to right,
	 * short-circuiting on the first non-SUCCESS result (AND logic).
	 *
	 * @param children the child nodes to evaluate in order
	 * @return a sequence composite node
	 */
	public static BehaviorNode sequence(BehaviorNode... children) {
		return new SequenceNode(Arrays.asList(children));
	}

	/**
	 * Creates a {@link SelectorNode} that ticks children left to right,
	 * short-circuiting on the first non-FAILURE result (priority OR logic).
	 * Children listed first have higher priority.
	 *
	 * @param children the child nodes to evaluate in priority order
	 * @return a selector composite node
	 */
	public static BehaviorNode selector(BehaviorNode... children) {
		return new SelectorNode(Arrays.asList(children));
	}

	/**
	 * Creates an {@link InverterNode} that flips SUCCESS↔FAILURE on its child
	 * (RUNNING is passed through unchanged).
	 *
	 * @param child the node whose result is inverted
	 * @return an inverter decorator node
	 */
	public static BehaviorNode invert(BehaviorNode child) {
		return new InverterNode(child);
	}

	/**
	 * Creates a node that executes {@code action} and immediately returns
	 * {@link NodeStatus#SUCCESS}. Useful for inserting one-shot side-effects
	 * (e.g. logging, telemetry) into a sequence.
	 *
	 * @param action the action to run
	 * @return an action node that always returns SUCCESS
	 */
	public static BehaviorNode runOnce(Runnable action) {
		return () -> {
			action.run();
			return NodeStatus.SUCCESS;
		};
	}
}
