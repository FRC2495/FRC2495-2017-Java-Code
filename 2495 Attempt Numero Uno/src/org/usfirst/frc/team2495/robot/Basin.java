package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Basin {
	CANTalon basin;
	static final double SCREW_PITCH_INCHES_PER_REV = .75;
	static final int LENGTH_OF_SCREW_INCHES = 8;
	boolean isHomingPart1, isHomingPart2, isMoving;
	final double REV_THRESH = .125;
	final double OFFSET_INCHES = 1;
	final double GEAR_RATIO = 187.0 / 2;
	double tac;
	boolean hasBeenHomed = false;

	public Basin(CANTalon basin_in) {
		basin = basin_in;
		basin.enableBrakeMode(true);
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		basin.enableLimitSwitch(false, true);
		basin.setInverted(false);
		basin.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		isHomingPart1 = false;
		isHomingPart2 = false;
		isMoving = false;
	}

	public boolean getLimitSwitchState() {
		return basin.isRevLimitSwitchClosed();
	}

	public void home() {
		hasBeenHomed = false;
		toVbs();
		if (!getLimitSwitchState()) {
			basin.set(.1);
			isHomingPart1 = true;
			isHomingPart2 = true;
		} else {
			isHomingPart1 = false;
			isHomingPart2 = true;
			basin.set(0);
			toEncPosition(4);
			basin.setPosition(0);
			basin.enableLimitSwitch(false, false);
			basin.enableControl();
			tac = -convertInchesToRev(OFFSET_INCHES);
			basin.set(tac);
			isHomingPart2 = true;
		}

	}

	public boolean checkHome() {
		if (isHomingPart1) {
			isHomingPart1 = !getLimitSwitchState();

			if (!isHomingPart1) {
				System.out.println("You have reached the home.");
				basin.set(0);
				toEncPosition(4);
				basin.setPosition(0);
				basin.enableLimitSwitch(false, false);
				basin.enableControl();
				tac = -convertInchesToRev(OFFSET_INCHES);
				basin.set(tac);
				isHomingPart2 = true;
			}
		} else if (isHomingPart2) {
			double enc = basin.getPosition();

			isHomingPart2 = !(enc > tac - REV_THRESH && enc < tac + REV_THRESH);

			if (!isHomingPart2) {
				System.out.println("You have reached the virtual zero.");
				hasBeenHomed = true;
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					System.out.println("HOW?!?!?!??!");
					e.printStackTrace();
				}
				toVbs();
				basin.setPosition(0);
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
			toEncPosition(4);
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
			toEncPosition(4);
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

	private void toEncPosition(int forward) {
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
