/*
package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Basin {
	CANTalon basin;
	
	static final double SCREW_PITCH_INCHES_PER_REV = .75;
	static final int LENGTH_OF_SCREW_INCHES = 11;
	
	boolean isHomingPart1, isHomingPart2, isMoving;
	
	static final double REV_THRESH = .125;
	static final double OFFSET_INCHES = 1;
	static final double GEAR_RATIO = 187.0 / 2;
	static final double HOMING_VOLTAGE = 0.1;
	static final double MOVING_VOLTAGE_VOLTS = 4.0;
	
	double tac;
	boolean hasBeenHomed = false;

	public Basin(CANTalon basin_in) {
		basin = basin_in;
		basin.enableBrakeMode(true); // enables break mode
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		basin.enableLimitSwitch(false, true); // enables limit switch only on reverse (i.e. bottom)
		basin.setInverted(false);
		basin.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative); // specifies encoder used
		isHomingPart1 = false;
		isHomingPart2 = false;
		isMoving = false;
	}

	// returns the state of the limit switch
	public boolean getLimitSwitchState() {
		return basin.isRevLimitSwitchClosed();
	}

	private void homePart1() {
		// assumes toVbs() already called
		basin.enableLimitSwitch(false, true); // enables limit switch only on reverse (i.e. bottom)
		basin.set(HOMING_VOLTAGE); // we start moving down
		isHomingPart1 = true;
	}
	
	private void homePart2() {
		basin.set(0); // we stop
		basin.setPosition(0); // we set the current position to zero	
		toEncPosition(MOVING_VOLTAGE_VOLTS); // we switch to position mode
		basin.enableLimitSwitch(false, false); // we disable stop on switch so we can move out
		basin.enableControl(); // we enable control
		tac = -convertInchesToRev(OFFSET_INCHES);
		basin.set(tac); // we move to virtual zero
		isHomingPart2 = true;
	}
	
	// homes the basin
	public void home() {
		hasBeenHomed = false; // flags that it has not been homed
		toVbs(); // switches to vbs
		
		if (!getLimitSwitchState()) { // if we are not already at the switch
			//isHomingPart1 = tru	e; // we need to go down to find limit switch					
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
			double enc = basin.getPosition();

			isHomingPart2 = !(enc > tac - REV_THRESH && enc < tac + REV_THRESH);

			if (!isHomingPart2) {
				System.out.println("You have reached the virtual zero.");
				try {
					Thread.sleep(100); // we wait a little for the screw to fully stop
				} catch (InterruptedException e) {
					System.out.println("HOW?!?!?!??!");
					e.printStackTrace();
				}
				toVbs(); // we switch back to vbus
				basin.setPosition(0); // we mark the virtual zero
				basin.enableLimitSwitch(false, true); // just in case
				hasBeenHomed = true;
			}
		}

		return isHoming();
	}

	public boolean checkMove() {
		if (isMoving) {
			double enc = basin.getPosition();

			isMoving = !(enc > tac - REV_THRESH && enc < tac + REV_THRESH);

			if (!isMoving) {
				System.out.println("You have reached the target (basin moving).");
				toVbs();
				basin.set(0);
			}
		}
		return isMoving;
	}

	public void moveUp() {
		if (hasBeenHomed) {
			toEncPosition(MOVING_VOLTAGE_VOLTS);
			System.out.println("Moving Up");
			basin.enableControl();
			tac = -convertInchesToRev(LENGTH_OF_SCREW_INCHES);
			basin.set(tac);
			isMoving = true;
		} else {
			System.out.println("You have not been home, your mother must be worried sick");
		}
	}

	public void moveDown() {
		if (hasBeenHomed) {
			toEncPosition(MOVING_VOLTAGE_VOLTS);
			System.out.println("Moving Down");
			basin.enableControl();
			tac = -convertInchesToRev(0);
			basin.set(tac);
			isMoving = true;
		} else {
			System.out.println("You have not been home, your mother must be worried sick");
		}
	}

	public double getPosition() {
		return basin.getPosition();
	}

	public double getEncPosition() {
		return basin.getEncPosition();
	}

	public boolean isHoming() {
		return isHomingPart1 || isHomingPart2;
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

	private void toVbs() {
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	private void toEncPosition(double forward) {
		basin.setPID(0.4, 0, 0);
		basin.changeControlMode(CANTalon.TalonControlMode.Position);
		basin.configPeakOutputVoltage(forward, -forward);
		basin.configNominalOutputVoltage(0, 0);
		basin.configNominalOutputVoltage(0, 0);
	}

	public double getTarget() {
		return tac;
	}
	
	public boolean hasBeenHomed()
	{
		return hasBeenHomed;
	}

}
*/