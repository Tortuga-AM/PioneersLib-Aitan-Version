package org.team7525.subsystem;

public interface SubsystemStates {
	/**
	 * Returns a human-readable name for this state.
	 *
	 * <p>For enum implementations this automatically returns the enum constant's name
	 * (e.g. {@code "IDLE"}, {@code "SCORING"}), so overriding is only required when a
	 * custom string is needed. Non-enum implementations must override this method.
	 */
	default String getStateString() {
		if (this instanceof Enum) {
			return ((Enum<?>) this).name();
		}
		throw new UnsupportedOperationException(
			"getStateString() must be overridden by non-enum implementations of SubsystemStates"
		);
	}
}
