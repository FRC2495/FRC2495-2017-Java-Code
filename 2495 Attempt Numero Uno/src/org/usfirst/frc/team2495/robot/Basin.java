package org.usfirst.frc.team2495.robot;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.ControlMode;

public class Basin {
	WPI_TalonSRX basin;
	
	static final double SCREW_PITCH_INCHES_PER_REV = .75;
	static final int LENGTH_OF_SCREW_INCHES = 11;
	
	boolean isHomingPart1, isHomingPart2, isMoving;
	
	static final double TICK_THRESH = 512;
	static final double OFFSET_INCHES = 1;
	static final double GEAR_RATIO = 187.0 / 2;
	static final double HOMING_PCT_OUTPUT = 0.1;
	static final double MAX_PCT_OUTPUT = 0.3;
	
	static final int PRIMARY_PID_LOOP = 0;
	static final int SLOT_0 = 0;
	static final int TALON_TIMEOUT_MS = 10;
	static final int TICKS_PER_REVOLUTION = 4096;
	
	double tac;
	boolean hasBeenHomed = false;

	public Basin(WPI_TalonSRX basin_in) {
		basin = basin_in;
		
		// Mode of operation during Neutral output may be set by using the setNeutralMode() function.
		// As of right now, there are two options when setting the neutral mode of a motor controller,
		// brake and coast.	
		basin.setNeutralMode(NeutralMode.Brake);
		
		// Sensor phase is the term used to explain sensor direction.
		// In order for limit switches and closed-loop features to function properly the sensor and motor has to be in-phase.
		// This means that the sensor position must move in a positive direction as the motor controller drives positive output.
		basin.setSensorPhase(false);

		//basin.enableLimitSwitch(false, true); // enables limit switch only on reverse (i.e. bottom)
		basin.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TALON_TIMEOUT_MS);
		basin.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, TALON_TIMEOUT_MS);
		basin.overrideLimitSwitchesEnable(true);

		// Motor controller output direction can be set by calling the setInverted() function as seen below.
		// Note: Regardless of invert value, the LEDs will blink green when positive output is requested (by robot code or firmware closed loop).
		// Only the motor leads are inverted. This feature ensures that sensor phase and limit switches will properly match the LED pattern
		// (when LEDs are green => forward limit switch and soft limits are being checked). 	
		basin.setInverted(false); // invert if required
		
		setNominalAndPeakOutputs(MAX_PCT_OUTPUT);

		// Sensors for motor controllers provide feedback about the position, velocity, and acceleration
		// of the system using that motor controller.
		// Note: With Phoenix framework, position units are in the natural units of the sensor.
		// This ensures the best resolution possible when performing closed-loops in firmware.
		// CTRE Magnetic Encoder (relative/quadrature) =  4096 units per rotation		
		basin.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,	PRIMARY_PID_LOOP, TALON_TIMEOUT_MS);
		
		isHomingPart1 = false;
		isHomingPart2 = false;
		isMoving = false;
	}

	// returns the state of the limit switch
	public boolean getLimitSwitchState() {
		return basin.getSensorCollection().isRevLimitSwitchClosed();
	}

	private void homePart1() {
		// assumes toVbs() already called
		//basin.enableLimitSwitch(false, true); // enables limit switch only on reverse (i.e. bottom)
		//basin.overrideLimitSwitchesEnable(true);
		basin.set(ControlMode.PercentOutput,-HOMING_PCT_OUTPUT); // we start moving down
		isHomingPart1 = true;
	}
	
	private void homePart2() {
		basin.set(ControlMode.PercentOutput,0); // we stop
		basin.setSelectedSensorPosition(0, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // we set the current position to zero	
		setPIDParameters(); // we switch to position mode
		//basin.enableLimitSwitch(false, false); // we disable stop on switch so we can move out
		//basin.overrideLimitSwitchesEnable(false);
		////basin.enableControl(); // we enable control
		tac = +convertInchesToRev(OFFSET_INCHES) * TICKS_PER_REVOLUTION;
		basin.set(ControlMode.Position,tac); // we move to virtual zero
		isHomingPart2 = true;
	}
	
	// homes the basin
	public void home() {
		hasBeenHomed = false; // flags that it has not been homed
		//toVbs(); // switches to vbs
		
		if (!getLimitSwitchState()) { // if we are not already at the switch
			//isHomingPart1 = true; // we need to go down to find limit switch					
			homePart1();
			isHomingPart2 = true; // then we need to go to virtual zero later
		} else {
			isHomingPart1 = false; // we don't need to go down
			//isHomingPart2 = true; // but we still need to go to virtual zero		
			homePart2(); // we start part 2 directly
		}
	}

	public boolean checkHome() {
		if (isHomingPart1) {
			isHomingPart1 = !getLimitSwitchState(); // we are not done until we reach the switch

			if (!isHomingPart1) {
				System.out.println("You have reached the home.");
				
				homePart2(); // we move on to part 2
			}
		} else if (isHomingPart2) {
			double enc = basin.getSelectedSensorPosition(PRIMARY_PID_LOOP);

			isHomingPart2 = !(enc > tac - TICK_THRESH && enc < tac + TICK_THRESH);

			if (!isHomingPart2) {
				System.out.println("You have reached the virtual zero.");
				try {
					Thread.sleep(100); // we wait a little for the screw to fully stop
				} catch (InterruptedException e) {
					System.out.println("HOW?!?!?!??!");
					e.printStackTrace();
				}
				//toVbs(); // we switch back to vbus
				basin.setSelectedSensorPosition(0, PRIMARY_PID_LOOP, TALON_TIMEOUT_MS); // we mark the virtual zero
				//basin.enableLimitSwitch(false, true); // just in case
				//basin.overrideLimitSwitchesEnable(true);
				hasBeenHomed = true;
			}
		}

		return isHoming();
	}

	public boolean checkMove() {
		if (isMoving) {
			double enc = basin.getSelectedSensorPosition(PRIMARY_PID_LOOP);

			isMoving = !(enc > tac - TICK_THRESH && enc < tac + TICK_THRESH);

			if (!isMoving) {
				System.out.println("You have reached the target (basin moving).");
				//toVbs();
				basin.set(ControlMode.PercentOutput,0);
			}
		}
		return isMoving;
	}

	public void moveUp() {
		if (hasBeenHomed) {
			setPIDParameters();
			System.out.println("Moving Up");
			//basin.enableControl();
			tac = +convertInchesToRev(LENGTH_OF_SCREW_INCHES) * TICKS_PER_REVOLUTION;
			basin.set(ControlMode.Position,tac);
			isMoving = true;
		} else {
			System.out.println("You have not been home, your mother must be worried sick");
		}
	}

	public void moveDown() {
		if (hasBeenHomed) {
			setPIDParameters();
			System.out.println("Moving Down");
			//basin.enableControl();
			tac = -convertInchesToRev(0)* TICKS_PER_REVOLUTION;
			basin.set(ControlMode.Position,tac);
			isMoving = true;
		} else {
			System.out.println("You have not been home, your mother must be worried sick");
		}
	}

	public double getPosition() {
		return basin.getSelectedSensorPosition(PRIMARY_PID_LOOP) / TICKS_PER_REVOLUTION;
	}

	public double getEncPosition() {
		return basin.getSelectedSensorPosition(PRIMARY_PID_LOOP);
	}

	public boolean isHoming() {
		return isHomingPart1 || isHomingPart2;
	}
	
	public boolean isHomingPart1() {
		return isHomingPart1;
	}
	
	public boolean isHomingPart2() {
		return isHomingPart2;
	}

	public boolean isMoving() {
		return isMoving;
	}

	private double convertInchesToRev(double inches) {
		return inches / SCREW_PITCH_INCHES_PER_REV * GEAR_RATIO;
	}

	private double convertRevtoInches(double rev) {
		return rev * SCREW_PITCH_INCHES_PER_REV / GEAR_RATIO;
	}

	private void setPIDParameters() {		
		basin.configAllowableClosedloopError(SLOT_0, 0, TALON_TIMEOUT_MS);
		
		// P is the proportional gain. It modifies the closed-loop output by a proportion (the gain value)
		// of the closed-loop error.
		// P gain is specified in output unit per error unit.
		// When tuning P, it's useful to estimate your starting value.
		// If you want your mechanism to drive 50% output when the error is 4096 (one rotation when using CTRE Mag Encoder),
		// then the calculated Proportional Gain would be (0.50 X 1023) / 4096 = ~0.125.
		
		// I is the integral gain. It modifies the closed-loop output according to the integral error
		// (summation of the closed-loop error each iteration).
		// I gain is specified in output units per integrated error.
		// If your mechanism never quite reaches your target and using integral gain is viable,
		// start with 1/100th of the Proportional Gain.
		
		// D is the derivative gain. It modifies the closed-loop output according to the derivative error
		// (change in closed-loop error each iteration).
		// D gain is specified in output units per derivative error.
		// If your mechanism accelerates too abruptly, Derivative Gain can be used to smooth the motion.
		// Typically start with 10x to 100x of your current Proportional Gain.
		
		basin.config_kP(SLOT_0, 0.4, TALON_TIMEOUT_MS);
		basin.config_kI(SLOT_0, 0, TALON_TIMEOUT_MS);
		basin.config_kD(SLOT_0, 0, TALON_TIMEOUT_MS);		
	}

	public void setNominalAndPeakOutputs(double peakOutput)
	{
		basin.configPeakOutputForward(peakOutput, TALON_TIMEOUT_MS);
		basin.configPeakOutputReverse(-peakOutput, TALON_TIMEOUT_MS);
		
		basin.configNominalOutputForward(0, TALON_TIMEOUT_MS);
		basin.configNominalOutputForward(0, TALON_TIMEOUT_MS);
	}
	
	public double getTarget() {
		return tac;
	}
	
	public boolean hasBeenHomed()
	{
		return hasBeenHomed;
	}

}
