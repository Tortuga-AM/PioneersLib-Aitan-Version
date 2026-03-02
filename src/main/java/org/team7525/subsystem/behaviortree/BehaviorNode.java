package org.team7525.subsystem.behaviortree;

/**
 * A single node in a behavior tree.
 *
 * <p>Every node is evaluated once per robot loop cycle via {@link #tick()}.
 * Composite nodes ({@link SequenceNode}, {@link SelectorNode}) recursively tick
 * their children. Leaf nodes ({@link ConditionNode}, {@link SetStateNode}) evaluate
 * a single condition or perform a single action.
 */
public interface BehaviorNode {

	/**
	 * Evaluates this node for one robot loop cycle.
	 *
	 * @return the result of this evaluation
	 */
	NodeStatus tick();
}
