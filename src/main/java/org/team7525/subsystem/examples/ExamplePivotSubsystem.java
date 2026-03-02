package org.team7525.subsystem.examples;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.RotationsPerSecond;

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
 *   <li>Define states as an enum implementing {@link SubsystemStates}</li>
 *   <li>Associate each enum value with a {@link SubsystemState} containing the target angle</li>
 *   <li>Register button-triggered transitions via {@code addTrigger}</li>
 *   <li>Use {@code stateInit()} for one-shot actions that happen on state entry</li>
 *   <li>Use {@code runState()} for continuous actions that repeat every loop</li>
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

		@Override
		public String getStateString() {
			return this.name();
		}
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

		// ── Transition triggers ────────────────────────────────────────────
		// A button: IDLE → INTAKING; B button: INTAKING → IDLE
		addTrigger(States.IDLE, States.INTAKING, driver.a()::getAsBoolean);
		addTrigger(States.INTAKING, States.IDLE, driver.b()::getAsBoolean);

		// Right trigger held → SCORING; releasing returns to IDLE from SCORING
		BooleanSupplier scoringHeld = () -> driver.getRightTriggerAxis() > 0.5;
		addTrigger(States.IDLE, States.SCORING, scoringHeld);
		addTrigger(States.INTAKING, States.SCORING, scoringHeld);
		addTrigger(States.SCORING, States.IDLE, () -> driver.getRightTriggerAxis() <= 0.5);
	}

	// ── Subsystem loop ────────────────────────────────────────────────────────

	@Override
	protected void runState() {
		// Move the arm toward the target angle for the active state
		double targetDegrees = getState().getSubsystemState().angularPosition().getDegrees();
		double currentDegrees = pivotEncoder.getDistance();
		double output = pidController.calculate(currentDegrees, targetDegrees);
		pivotMotor.set(output);
	}

	// ── State lifecycle hooks ─────────────────────────────────────────────────

	@Override
	protected void stateInit() {
		// Reset the PID accumulator every time we enter a new state to avoid integral windup
		pidController.reset();
	}

	// ── Public helpers ────────────────────────────────────────────────────────

	/** Returns {@code true} when the arm is within 2° of its target position. */
	public boolean atTargetAngle() {
		double targetDegrees = getState().getSubsystemState().angularPosition().getDegrees();
		return Math.abs(pivotEncoder.getDistance() - targetDegrees) < 2.0;
	}
}
