package org.team7525.subsystem.examples;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import java.util.function.BooleanSupplier;
import org.team7525.subsystem.Subsystem;
import org.team7525.subsystem.SubsystemState;
import org.team7525.subsystem.SubsystemStates;

/**
 * Example pivot (arm) subsystem demonstrating how to use the PioneersLib state machine.
 *
 * <p>This subsystem controls a single-jointed arm that can be in one of three states:
 * IDLE (stowed), INTAKING (low position), or SCORING (high position). Controller button
 * triggers drive the transitions.
 *
 * <p>Key patterns demonstrated:
 * <ul>
 *   <li>Define states as an enum implementing {@link SubsystemStates} – {@code getStateString()}
 *       is provided automatically, no override needed</li>
 *   <li>Associate each enum value with a {@link SubsystemState} containing the target angle</li>
 *   <li>Register transitions fluently via {@link #fromState(SubsystemStates)}</li>
 *   <li>Use {@link #addStateEntryAction(SubsystemStates, Runnable)} for per-state entry logic
 *       instead of a switch in {@code stateInit()}</li>
 *   <li>Use {@code runState()} for continuous hardware commands each loop cycle</li>
 * </ul>
 */
public class ExamplePivotSubsystem extends Subsystem<ExamplePivotSubsystem.States> {

	// ── Hardware ─────────────────────────────────────────────────────────────
	private final PWMSparkMax pivotMotor = new PWMSparkMax(0);
	private final Encoder pivotEncoder = new Encoder(0, 1);
	private final PIDController pidController = new PIDController(0.05, 0.0, 0.001);

	// ── State definitions ────────────────────────────────────────────────────

	public enum States implements SubsystemStates {
		/** Arm stowed upright – safe for driving. */
		IDLE {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromPivotStates("IDLE", Rotation2d.fromDegrees(90));
			}
		},

		/** Arm lowered to pick up game pieces from the floor. */
		INTAKING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromPivotStates("INTAKING", Rotation2d.fromDegrees(0));
			}
		},

		/** Arm raised to score in the high goal. */
		SCORING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromPivotStates("SCORING", Rotation2d.fromDegrees(135));
			}
		};

		/** Returns the hardware setpoints associated with this state. */
		public abstract SubsystemState getSubsystemState();

		// getStateString() is inherited automatically from SubsystemStates (returns name())
	}

	// ── Constructor ───────────────────────────────────────────────────────────

	/**
	 * Constructs the pivot subsystem and wires up state-transition triggers.
	 *
	 * @param driver the driver's Xbox controller used to command state transitions
	 */
	public ExamplePivotSubsystem(CommandXboxController driver) {
		super("ExamplePivot", States.IDLE);

		// Encoder returns degrees; convert to rotations for PID
		pivotEncoder.setDistancePerPulse(360.0 / 2048.0);

		// ── Transitions (fluent builder) ───────────────────────────────────
		BooleanSupplier scoringHeld = () -> driver.getRightTriggerAxis() > 0.5;

		fromState(States.IDLE)
				.goTo(States.INTAKING, driver.a()::getAsBoolean)
				.goTo(States.SCORING,  scoringHeld);

		fromState(States.INTAKING)
				.goTo(States.IDLE,    driver.b()::getAsBoolean)
				.goTo(States.SCORING, scoringHeld);

		fromState(States.SCORING)
				.goTo(States.IDLE, () -> driver.getRightTriggerAxis() <= 0.5);

		// ── Per-state entry actions ────────────────────────────────────────
		// Reset PID every time we enter any state to avoid integral windup.
		// addStateEntryAction is called once per state-entry; no override needed.
		addStateEntryAction(States.IDLE,     pidController::reset);
		addStateEntryAction(States.INTAKING, pidController::reset);
		addStateEntryAction(States.SCORING,  pidController::reset);
	}

	// ── Subsystem loop ────────────────────────────────────────────────────────

	@Override
	protected void runState() {
		// Move the arm toward the target angle for the active state
		double targetDegrees = getState().getSubsystemState().angularPosition().getDegrees();
		double currentDegrees = pivotEncoder.getDistance();
		pivotMotor.set(pidController.calculate(currentDegrees, targetDegrees));
	}

	// ── Public helpers ────────────────────────────────────────────────────────

	/** Returns {@code true} when the arm is within 2° of its target position. */
	public boolean atTargetAngle() {
		double targetDegrees = getState().getSubsystemState().angularPosition().getDegrees();
		return Math.abs(pivotEncoder.getDistance() - targetDegrees) < 2.0;
	}

	/** Returns {@code true} if the arm is currently in the scoring position. */
	public boolean isScoring() {
		return isInState(States.SCORING);
	}
}
