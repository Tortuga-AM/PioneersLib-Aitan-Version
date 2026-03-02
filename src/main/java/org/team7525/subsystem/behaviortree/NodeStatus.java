package org.team7525.subsystem.behaviortree;

/** The result of ticking a single behavior tree node for one robot loop cycle. */
public enum NodeStatus {
	/** The node completed its work successfully. */
	SUCCESS,

	/** The node could not complete its work (condition not met, etc.). */
	FAILURE,

	/**
	 * The node is still executing and needs to be ticked again next loop.
	 * Action nodes (e.g. {@code go(state)}) return this value indefinitely because
	 * hardware control is a continuous, ongoing process.
	 */
	RUNNING
}
