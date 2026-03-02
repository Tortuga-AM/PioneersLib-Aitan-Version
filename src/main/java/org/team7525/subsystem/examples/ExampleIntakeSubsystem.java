package org.team7525.subsystem.examples;

import static edu.wpi.first.units.Units.RotationsPerSecond;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team7525.subsystem.Subsystem;
import org.team7525.subsystem.SubsystemState;
import org.team7525.subsystem.SubsystemStates;

/**
 * Example intake subsystem demonstrating combined pivot + roller control with the PioneersLib
 * state machine.
 *
 * <p>The intake has a pivot arm (opens/closes) and a roller (spins to grab or eject game pieces).
 * A beam-break sensor detects when a piece has been captured, triggering an automatic transition
 * from INTAKING to HOLDING without any driver input.
 *
 * <p>Key patterns demonstrated:
 * <ul>
 *   <li>Combine pivot angle and roller speed in a single {@link SubsystemState} via
 *       {@link SubsystemState#fromOpeningIntakeStates}</li>
 *   <li>Mix driver-commanded transitions with autonomous sensor-driven transitions</li>
 *   <li>Use {@link #addStateExitAction(SubsystemStates, Runnable)} to clear rumble on exit</li>
 *   <li>Use {@link #addRunnableTrigger(Runnable, java.util.function.BooleanSupplier)} for
 *       side-effects that don't change state (e.g. rumble while holding a piece)</li>
 *   <li>Use {@link #addTransitionCallback(SubsystemStates, SubsystemStates, Runnable)} to
 *       react to a specific transition (INTAKING → HOLDING)</li>
 * </ul>
 */
public class ExampleIntakeSubsystem extends Subsystem<ExampleIntakeSubsystem.States> {

	// ── Hardware ─────────────────────────────────────────────────────────────
	private final CommandXboxController driver;
	private final PWMSparkMax pivotMotor = new PWMSparkMax(2);
	private final PWMSparkMax rollerMotor = new PWMSparkMax(3);
	private final Encoder pivotEncoder = new Encoder(4, 5);
	private final DigitalInput beamBreak = new DigitalInput(6);
	private final PIDController pivotPID = new PIDController(0.04, 0.0, 0.0005);

	// ── State definitions ────────────────────────────────────────────────────

	public enum States implements SubsystemStates {
		/**
		 * Intake retracted and rollers stopped – safe resting position.
		 * Pivot: 90°, roller: 0 RPS.
		 */
		STOWED {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"STOWED", Rotation2d.fromDegrees(90), RotationsPerSecond.of(0));
			}
		},

		/**
		 * Intake deployed and rollers spinning inward to collect a game piece.
		 * Pivot: 10°, roller: 30 RPS.
		 */
		INTAKING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"INTAKING", Rotation2d.fromDegrees(10), RotationsPerSecond.of(30));
			}
		},

		/**
		 * Intake slightly retracted with rollers idling to keep the piece secure.
		 * Pivot: 45°, roller: 5 RPS (holding tension).
		 */
		HOLDING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"HOLDING", Rotation2d.fromDegrees(45), RotationsPerSecond.of(5));
			}
		},

		/**
		 * Rollers reversed to eject the game piece.
		 * Pivot: 30°, roller: -40 RPS.
		 */
		EJECTING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"EJECTING", Rotation2d.fromDegrees(30), RotationsPerSecond.of(-40));
			}
		};

		/** Returns the hardware setpoints associated with this state. */
		public abstract SubsystemState getSubsystemState();

		// getStateString() is inherited automatically from SubsystemStates (returns name())
	}

	// ── Constructor ───────────────────────────────────────────────────────────

	/**
	 * Constructs the intake subsystem and registers all state-transition triggers.
	 *
	 * @param driver the driver's Xbox controller used to command intake actions
	 */
	public ExampleIntakeSubsystem(CommandXboxController driver) {
		super("ExampleIntake", States.STOWED);
		this.driver = driver;

		pivotEncoder.setDistancePerPulse(360.0 / 4096.0); // degrees per pulse

		// ── Driver-commanded transitions ───────────────────────────────────
		fromState(States.STOWED)
				.goTo(States.INTAKING, driver.leftBumper()::getAsBoolean)
				.goTo(States.EJECTING, () -> driver.getLeftTriggerAxis() > 0.5);

		fromState(States.INTAKING)
				.goTo(States.STOWED, () -> !driver.leftBumper().getAsBoolean());

		fromState(States.HOLDING)
				.goTo(States.STOWED,   driver.rightBumper()::getAsBoolean)
				.goTo(States.EJECTING, () -> driver.getLeftTriggerAxis() > 0.5);

		fromState(States.EJECTING)
				.goTo(States.STOWED, () -> driver.getLeftTriggerAxis() <= 0.5);

		// ── Sensor-driven transition ───────────────────────────────────────
		// Beam-break (active-low) fires automatically when a piece is captured
		addTrigger(States.INTAKING, States.HOLDING, () -> !beamBreak.get());

		// ── Per-state exit actions ─────────────────────────────────────────
		// Reset PID to prevent windup on every state exit
		addStateExitAction(States.STOWED,   pivotPID::reset);
		addStateExitAction(States.INTAKING, pivotPID::reset);
		addStateExitAction(States.HOLDING,  pivotPID::reset);
		addStateExitAction(States.EJECTING, pivotPID::reset);

		// Clear controller rumble when leaving HOLDING so it doesn't persist
		addStateExitAction(States.HOLDING,
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0));

		// ── Transition callback ────────────────────────────────────────────
		// Log and rumble the moment a piece is captured (INTAKING → HOLDING)
		addTransitionCallback(States.INTAKING, States.HOLDING, () -> {
			System.out.println("[Intake] Game piece captured!");
			driver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
		});

		// ── Side-effect trigger ────────────────────────────────────────────
		// Keep rumble active every loop while holding a piece (the exit action stops it)
		addRunnableTrigger(
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 1.0),
				() -> isInState(States.HOLDING)
		);
	}

	// ── Subsystem loop ────────────────────────────────────────────────────────

	@Override
	protected void runState() {
		SubsystemState target = getState().getSubsystemState();

		// Drive pivot to the target angle
		double targetDegrees = target.angularPosition().getDegrees();
		double currentDegrees = pivotEncoder.getDistance();
		pivotMotor.set(pivotPID.calculate(currentDegrees, targetDegrees));

		// Drive roller to the target speed (open-loop percent from RPS)
		AngularVelocity targetRollerVel = target.angularVelocity();
		double rollerPercent = targetRollerVel.in(RotationsPerSecond) / 50.0; // ~50 RPS free speed
		rollerMotor.set(rollerPercent);
	}

	// ── Public helpers ────────────────────────────────────────────────────────

	/** Returns {@code true} when the beam-break sensor detects a game piece. */
	public boolean hasPiece() {
		return !beamBreak.get();
	}

	/** Returns {@code true} if the intake is actively holding a game piece. */
	public boolean isHolding() {
		return isInState(States.HOLDING);
	}
}
