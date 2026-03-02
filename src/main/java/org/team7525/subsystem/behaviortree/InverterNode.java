package org.team7525.subsystem.behaviortree;

/**
 * A decorator node that inverts its child's result.
 *
 * <ul>
 *   <li>{@link NodeStatus#SUCCESS} → {@link NodeStatus#FAILURE}</li>
 *   <li>{@link NodeStatus#FAILURE} → {@link NodeStatus#SUCCESS}</li>
 *   <li>{@link NodeStatus#RUNNING} → {@link NodeStatus#RUNNING} (unchanged)</li>
 * </ul>
 *
 * <p>Useful for expressing "condition is NOT met" inside a sequence:
 * <pre>{@code
 * // Transition to STOWED when the left bumper is released
 * BT.sequence(BT.invert(BT.condition(driver.leftBumper()::getAsBoolean)), go(States.STOWED))
 * }</pre>
 *
 * <p>Create via {@link BT#invert(BehaviorNode)}.
 */
public class InverterNode implements BehaviorNode {

	private final BehaviorNode child;

	InverterNode(BehaviorNode child) {
		this.child = child;
	}

	@Override
	public NodeStatus tick() {
		return switch (child.tick()) {
			case SUCCESS -> NodeStatus.FAILURE;
			case FAILURE -> NodeStatus.SUCCESS;
			case RUNNING -> NodeStatus.RUNNING;
		};
	}
}
