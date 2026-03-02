package org.team7525.subsystem.behaviortree;

import java.util.List;

/**
 * A composite node that ticks its children left to right and succeeds only when
 * <em>all</em> children succeed — AND logic.
 *
 * <p>Evaluation rules per tick:
 * <ol>
 *   <li>Tick each child in order.</li>
 *   <li>If a child returns {@link NodeStatus#FAILURE} or {@link NodeStatus#RUNNING},
 *       return that status immediately (short-circuit).</li>
 *   <li>If every child returns {@link NodeStatus#SUCCESS}, return SUCCESS.</li>
 * </ol>
 *
 * <p>The tree is re-evaluated from the root every loop cycle, so there is no
 * "memory" of which child was last running. This gives fully reactive, priority-
 * respecting behavior: a higher-priority branch can preempt a lower-priority one
 * the moment its conditions become true.
 *
 * <p>Typical use: gate an action behind one or more conditions.
 * <pre>{@code
 * BT.sequence(
 *     BT.condition(driver.a()::getAsBoolean),  // must be pressing A
 *     go(States.INTAKING)                       // → set state
 * )
 * }</pre>
 *
 * <p>Create via {@link BT#sequence(BehaviorNode...)}.
 */
public class SequenceNode implements BehaviorNode {

	private final List<BehaviorNode> children;

	SequenceNode(List<BehaviorNode> children) {
		this.children = children;
	}

	@Override
	public NodeStatus tick() {
		for (BehaviorNode child : children) {
			NodeStatus status = child.tick();
			if (status != NodeStatus.SUCCESS) {
				// Stop on first FAILURE or RUNNING (short-circuit AND)
				return status;
			}
		}
		return NodeStatus.SUCCESS;
	}
}
