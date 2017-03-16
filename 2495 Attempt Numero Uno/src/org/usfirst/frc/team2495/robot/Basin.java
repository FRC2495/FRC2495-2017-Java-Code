package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;

public class Basin {
	CANTalon basin;
	static final double SCREW_PITCH_INCHES_PER_REV = .75; 
	static final int LENGTH_OF_SCREW_INCHES = 13;

	public Basin(CANTalon basin_in) {
		basin = basin_in;
		basin.enableBrakeMode(true);
		basin.changeControlMode(CANTalon.TalonControlMode.Position);
		basin.enableLimitSwitch(false, true);
	}

	public boolean getLimitSwitchState() {
		return basin.isRevLimitSwitchClosed();
	}

	public void home() {
		toVbs();
		if (!getLimitSwitchState()) {
			basin.set(-.1);
		}
		else
		{
			basin.set(0);
		}
		
		basin.setEncPosition(0);
	}
	
	public void moveUp()
	{
		toEncPosition();
		basin.set(convertInchesToRev(LENGTH_OF_SCREW_INCHES));
	}
	
	public void moveDown()
	{
		basin.set(convertInchesToRev(-LENGTH_OF_SCREW_INCHES));
	}
	
	private double convertInchesToRev (double inches)
	{
		return inches / SCREW_PITCH_INCHES_PER_REV;
	}

	private void toVbs() {
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	private void toEncPosition() {
		basin.changeControlMode(CANTalon.TalonControlMode.Position);
	}

}
