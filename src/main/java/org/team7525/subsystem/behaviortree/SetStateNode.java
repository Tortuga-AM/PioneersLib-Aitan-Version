package org.team7525.subsystem.behaviortree;

import java.util.function.Consumer;
import org.team7525.subsystem.SubsystemStates;

/**
 * A leaf action node that drives a subsystem into a target state.
 *
 * <p>Each time this node is ticked it calls {@code setState(targetState)} on the
 * enclosing subsystem and returns {@link NodeStatus#RUNNING} — because hardware
 * control is a continuous, ongoing operation rather than an instantaneous one.
 *
 * <p>The {@code setState()} no-op guard ({@code if (state == newState) return;})
 * ensures that lifecycle hooks fire only on an <em>actual</em> state change, not on
 * every tick.
 *
 * <p>Instances are created exclusively via {@code Subsystem.go(targetState)} to
 * keep the {@code Consumer<S>} reference internal.
 */
class SetStateNode<S extends SubsystemStates> implements BehaviorNode {

	private final Consumer<S> setStateFn;
	private final S targetState;

	SetStateNode(Consumer<S> setStateFn, S targetState) {
		this.setStateFn = setStateFn;
		this.targetState = targetState;
	}

	@Override
	public NodeStatus tick() {
		setStateFn.accept(targetState);
		return NodeStatus.RUNNING;
	}
}
