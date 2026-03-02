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
import org.team7525.subsystem.behaviortree.BT;

/**
 * The same intake as {@link ExampleIntakeSubsystem}, rewritten using a
 * <em>behavior tree</em> instead of a state machine.
 *
 * <h2>Why a behavior tree?</h2>
 *
 * <p>The state machine in {@link ExampleIntakeSubsystem} required <b>7 explicit
 * transitions</b> plus an {@code addTriggerFromAny()} call to express the intake's
 * logic. The equivalent behavior tree below uses <b>5 priority rules</b> and needs
 * no {@code addTrigger*} calls at all — the selector's top-to-bottom ordering
 * <em>is</em> the transition graph.
 *
 * <pre>
 * State machine (8 calls)                Behavior tree (1 call)
 * ─────────────────────────────────────  ──────────────────────────────────────────────
 * addTrigger(STOWED, INTAKING, …)        setBehaviorTree(
 * addTrigger(INTAKING, STOWED, …)            BT.selector(
 * addTrigger(INTAKING, HOLDING, …)               seq(back,       go(ESTOP)),   ← #1
 * addTrigger(HOLDING, EJECTING, …)               seq(leftTrig,   go(EJECTING)),← #2
 * addTrigger(HOLDING, STOWED, …)                 seq(!beamBreak, go(HOLDING)), ← #3
 * addTrigger(EJECTING, STOWED, …)                seq(leftBumper, go(INTAKING)),← #4
 * addTrigger(STOWED, EJECTING, …)                go(STOWED)                    ← fallback
 * addTriggerFromAny(ESTOP, …)               )
 *                                        );
 * </pre>
 *
 * <h2>How the selector works each loop</h2>
 * <ol>
 *   <li>Check child #1 (e-stop sequence): if back button is held, transition to ESTOP
 *       and stop evaluating. This is equivalent to {@code addTriggerFromAny} — it fires
 *       regardless of the current state.</li>
 *   <li>Check child #2: if left trigger &gt; 0.5, go to EJECTING.</li>
 *   <li>Check child #3: if beam-break is active, go to HOLDING.</li>
 *   <li>Check child #4: if left bumper is held, go to INTAKING.</li>
 *   <li>Fallback: go to STOWED.</li>
 * </ol>
 *
 * <p>Because the tree is re-evaluated from the root every loop, a higher-priority
 * condition always preempts a lower one the moment it becomes true — without any
 * explicit transition for that specific pair of states.
 *
 * <h2>When to prefer the state machine instead</h2>
 * <p>For simpler subsystems (e.g. a pivot arm with 2–3 states and straightforward
 * button transitions) the state machine's explicit A→B style is easier to read and
 * debug. Use the behavior tree when you have many overlapping conditions, need
 * global preemption, or want priorities to <em>replace</em> a large transition table.
 */
public class ExampleBTIntakeSubsystem extends Subsystem<ExampleBTIntakeSubsystem.States> {

	// ── Hardware ─────────────────────────────────────────────────────────────
	private final CommandXboxController driver;
	private final PWMSparkMax pivotMotor = new PWMSparkMax(2);
	private final PWMSparkMax rollerMotor = new PWMSparkMax(3);
	private final Encoder pivotEncoder = new Encoder(4, 5);
	private final DigitalInput beamBreak = new DigitalInput(6);
	private final PIDController pivotPID = new PIDController(0.04, 0.0, 0.0005);

	// ── State definitions ────────────────────────────────────────────────────

	public enum States implements SubsystemStates {
		/** Intake retracted and rollers stopped – safe resting position. */
		STOWED {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"STOWED", Rotation2d.fromDegrees(90), RotationsPerSecond.of(0));
			}
		},

		/** Intake deployed, rollers spinning inward to collect a game piece. */
		INTAKING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"INTAKING", Rotation2d.fromDegrees(10), RotationsPerSecond.of(30));
			}
		},

		/** Intake retracted, rollers idling to keep the piece secure. */
		HOLDING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"HOLDING", Rotation2d.fromDegrees(45), RotationsPerSecond.of(5));
			}
		},

		/** Rollers reversed to eject the game piece. */
		EJECTING {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"EJECTING", Rotation2d.fromDegrees(30), RotationsPerSecond.of(-40));
			}
		},

		/** Emergency stop – motors cut. Reachable from any state via back button. */
		ESTOP {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromOpeningIntakeStates(
						"ESTOP", Rotation2d.fromDegrees(90), RotationsPerSecond.of(0));
			}
		};

		public abstract SubsystemState getSubsystemState();
		// getStateString() auto-derived from enum name()
	}

	// ── Constructor ───────────────────────────────────────────────────────────

	public ExampleBTIntakeSubsystem(CommandXboxController driver) {
		super("ExampleBTIntake", States.STOWED);
		this.driver = driver;

		pivotEncoder.setDistancePerPulse(360.0 / 4096.0);

		// ── Behavior tree ──────────────────────────────────────────────────
		//
		// The root selector evaluates each branch top-to-bottom and activates the
		// first one whose conditions are met. This replaces all addTrigger() calls.
		//
		//   Priority 1 (highest): e-stop  — back button pressed
		//   Priority 2: ejecting          — left trigger held
		//   Priority 3: holding           — beam-break sensor triggered
		//   Priority 4: intaking          — left bumper held
		//   Priority 5 (fallback): stow   — always matches if nothing else does
		setBehaviorTree(
			BT.selector(
				BT.sequence(BT.condition(driver.back()::getAsBoolean),
					go(States.ESTOP)),
				BT.sequence(BT.condition(() -> driver.getLeftTriggerAxis() > 0.5),
					go(States.EJECTING)),
				BT.sequence(BT.condition(() -> !beamBreak.get()),
					go(States.HOLDING)),
				BT.sequence(BT.condition(driver.leftBumper()::getAsBoolean),
					go(States.INTAKING)),
				go(States.STOWED)
			)
		);

		// ── Lifecycle actions (work unchanged with behavior tree) ──────────
		// Reset PID on every state exit to prevent windup
		addStateExitAction(States.STOWED,   pivotPID::reset);
		addStateExitAction(States.INTAKING, pivotPID::reset);
		addStateExitAction(States.HOLDING,  pivotPID::reset);
		addStateExitAction(States.EJECTING, pivotPID::reset);

		// Clear rumble when leaving HOLDING
		addStateExitAction(States.HOLDING,
				() -> driver.getHID().setRumble(RumbleType.kBothRumble, 0.0));

		// Log and rumble the moment a piece is captured (transition callback still works)
		addTransitionCallback(States.INTAKING, States.HOLDING, () ->
				driver.getHID().setRumble(RumbleType.kBothRumble, 1.0));
	}

	// ── Subsystem loop ────────────────────────────────────────────────────────

	@Override
	protected void runState() {
		if (isInState(States.ESTOP)) {
			pivotMotor.set(0.0);
			rollerMotor.set(0.0);
			return;
		}

		SubsystemState target = getState().getSubsystemState();

		double targetDegrees = target.angularPosition().getDegrees();
		pivotMotor.set(pivotPID.calculate(pivotEncoder.getDistance(), targetDegrees));

		AngularVelocity targetRollerVel = target.angularVelocity();
		rollerMotor.set(targetRollerVel.in(RotationsPerSecond) / 50.0);
	}

	// ── Public helpers ────────────────────────────────────────────────────────

	/** Returns {@code true} when the beam-break sensor detects a game piece. */
	public boolean hasPiece() {
		return !beamBreak.get();
	}
}
