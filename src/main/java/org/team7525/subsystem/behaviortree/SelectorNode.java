package org.team7525.subsystem.behaviortree;

import java.util.List;

/**
 * A composite node that tries its children left to right and succeeds as soon as
 * <em>any</em> child succeeds — priority OR logic.
 *
 * <p>Evaluation rules per tick:
 * <ol>
 *   <li>Tick each child in order.</li>
 *   <li>If a child returns {@link NodeStatus#SUCCESS} or {@link NodeStatus#RUNNING},
 *       return that status immediately (short-circuit).</li>
 *   <li>If every child returns {@link NodeStatus#FAILURE}, return FAILURE.</li>
 * </ol>
 *
 * <p>The tree is re-evaluated from the root every loop cycle.  Children listed
 * <em>first</em> have the highest priority: if their conditions are met they
 * preempt all children listed after them.
 *
 * <p>Typical use: express a list of behaviours in priority order with a fallback.
 * <pre>{@code
 * BT.selector(
 *     BT.sequence(BT.condition(operator::getBackButton), go(States.ESTOP)),   // #1 priority
 *     BT.sequence(BT.condition(() -> trig > 0.5),        go(States.EJECTING)), // #2
 *     BT.sequence(BT.condition(() -> !beamBreak.get()),  go(States.HOLDING)),  // #3
 *     BT.sequence(BT.condition(driver::getLeftBumper),   go(States.INTAKING)), // #4
 *     go(States.STOWED)                                                         // fallback
 * )
 * }</pre>
 *
 * <p>Create via {@link BT#selector(BehaviorNode...)}.
 */
public class SelectorNode implements BehaviorNode {

	private final List<BehaviorNode> children;

	SelectorNode(List<BehaviorNode> children) {
		this.children = children;
	}

	@Override
	public NodeStatus tick() {
		for (BehaviorNode child : children) {
			NodeStatus status = child.tick();
			if (status != NodeStatus.FAILURE) {
				// Stop on first SUCCESS or RUNNING (short-circuit OR)
				return status;
			}
		}
		return NodeStatus.FAILURE;
	}
}
