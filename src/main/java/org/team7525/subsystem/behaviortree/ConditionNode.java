package org.team7525.subsystem.behaviortree;

import java.util.function.BooleanSupplier;

/**
 * A leaf node that evaluates a boolean condition.
 *
 * <p>Returns {@link NodeStatus#SUCCESS} when the condition is {@code true},
 * {@link NodeStatus#FAILURE} when it is {@code false}. Never returns RUNNING.
 *
 * <p>Create via {@link BT#condition(BooleanSupplier)}.
 */
public class ConditionNode implements BehaviorNode {

	private final BooleanSupplier condition;

	ConditionNode(BooleanSupplier condition) {
		this.condition = condition;
	}

	@Override
	public NodeStatus tick() {
		return condition.getAsBoolean() ? NodeStatus.SUCCESS : NodeStatus.FAILURE;
	}
}
