package org.team7525.subsystem.examples;

import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import org.team7525.subsystem.Subsystem;
import org.team7525.subsystem.SubsystemState;
import org.team7525.subsystem.SubsystemStates;

/**
 * Example elevator subsystem demonstrating profiled-PID motion with the PioneersLib state machine.
 *
 * <p>The elevator has four heights: BOTTOM, LOW, MID, and HIGH. The driver selects a
 * height with the D-pad; the elevator profiles smoothly to the requested position. An
 * E-stop (emergency-stop) state is also included to show how {@code addTriggerFromAny()}
 * can be used for transitions that must be reachable from every other state.
 *
 * <p>Key patterns demonstrated:
 * <ul>
 *   <li>Store height setpoints via {@link SubsystemState#fromElevatorStates}</li>
 *   <li>Use a profiled PID + feedforward for smooth trapezoid motion</li>
 *   <li>Use {@code addTriggerFromAny()} for a global e-stop transition</li>
 *   <li>Use {@code stateInit()} to reset the motion profile on each state change</li>
 * </ul>
 */
public class ExampleElevatorSubsystem extends Subsystem<ExampleElevatorSubsystem.States> {

	// ── Hardware ─────────────────────────────────────────────────────────────
	private final PWMSparkMax elevatorMotor = new PWMSparkMax(1);
	private final Encoder elevatorEncoder = new Encoder(2, 3);

	// Motion constraints: max 1.5 m/s, max 3 m/s²
	private static final TrapezoidProfile.Constraints CONSTRAINTS =
			new TrapezoidProfile.Constraints(1.5, 3.0);

	private final ProfiledPIDController pidController =
			new ProfiledPIDController(5.0, 0.0, 0.1, CONSTRAINTS);

	// kS=0.1 V static friction, kG=0.5 V gravity, kV=1.0 V/(m/s)
	private final ElevatorFeedforward feedforward = new ElevatorFeedforward(0.1, 0.5, 1.0);

	// ── State definitions ────────────────────────────────────────────────────

	public enum States implements SubsystemStates {
		/** Elevator fully lowered. */
		BOTTOM {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromElevatorStates("BOTTOM", Meters.of(0.0), MetersPerSecond.of(0));
			}
		},

		/** Low scoring position. */
		LOW {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromElevatorStates("LOW", Meters.of(0.3), MetersPerSecond.of(0));
			}
		},

		/** Mid scoring position. */
		MID {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromElevatorStates("MID", Meters.of(0.6), MetersPerSecond.of(0));
			}
		},

		/** High scoring position. */
		HIGH {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromElevatorStates("HIGH", Meters.of(1.0), MetersPerSecond.of(0));
			}
		},

		/**
		 * Emergency-stop state: motor is cut immediately.
		 * Reachable from any other state via the back button.
		 */
		ESTOP {
			@Override
			public SubsystemState getSubsystemState() {
				return SubsystemState.fromElevatorStates("ESTOP", Meters.of(0.0), MetersPerSecond.of(0));
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
	 * Constructs the elevator subsystem and registers transition triggers.
	 *
	 * @param operator the operator's Xbox controller used to command height changes
	 */
	public ExampleElevatorSubsystem(CommandXboxController operator) {
		super("ExampleElevator", States.BOTTOM);

		// Encoder: 2048 pulses/revolution, 2 cm/revolution (lead-screw pitch)
		elevatorEncoder.setDistancePerPulse(0.02 / 2048.0);

		pidController.setTolerance(0.01); // 1 cm tolerance

		// ── Height selection (D-pad) ───────────────────────────────────────
		addTrigger(States.BOTTOM, States.LOW, operator.povUp()::getAsBoolean);
		addTrigger(States.LOW, States.MID, operator.povUp()::getAsBoolean);
		addTrigger(States.MID, States.HIGH, operator.povUp()::getAsBoolean);

		addTrigger(States.HIGH, States.MID, operator.povDown()::getAsBoolean);
		addTrigger(States.MID, States.LOW, operator.povDown()::getAsBoolean);
		addTrigger(States.LOW, States.BOTTOM, operator.povDown()::getAsBoolean);

		// ── Global e-stop (back button transitions from ANY state) ─────────
		// addTriggerFromAny ensures this fires no matter what state we're in.
		addTriggerFromAny(States.ESTOP, operator.back()::getAsBoolean);
	}

	// ── Subsystem loop ────────────────────────────────────────────────────────

	@Override
	protected void runState() {
		if (getState() == States.ESTOP) {
			elevatorMotor.set(0.0);
			return;
		}

		Distance targetHeight = getState().getSubsystemState().position();
		double currentHeight = elevatorEncoder.getDistance();

		double pidOutput = pidController.calculate(currentHeight, targetHeight.in(Meters));
		double ffOutput = feedforward.calculate(pidController.getSetpoint().velocity);
		elevatorMotor.setVoltage(pidOutput + ffOutput);
	}

	// ── State lifecycle hooks ─────────────────────────────────────────────────

	@Override
	protected void stateInit() {
		if (getState() == States.ESTOP) return;

		// Seed the profiled PID with the current position so there is no discontinuous jump
		Distance targetHeight = getState().getSubsystemState().position();
		pidController.reset(elevatorEncoder.getDistance());
		pidController.setGoal(targetHeight.in(Meters));
	}

	// ── Public helpers ────────────────────────────────────────────────────────

	/** Returns {@code true} when the elevator is within tolerance of its target height. */
	public boolean atTargetHeight() {
		return pidController.atGoal();
	}

	/** Returns the current elevator height in meters. */
	public double getHeightMeters() {
		return elevatorEncoder.getDistance();
	}
}
