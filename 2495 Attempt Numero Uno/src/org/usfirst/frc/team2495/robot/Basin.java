package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Basin {
	CANTalon basin;
	static final double SCREW_PITCH_INCHES_PER_REV = .75; 
	static final int LENGTH_OF_SCREW_INCHES = 13;
	boolean isHoming;

	public Basin(CANTalon basin_in) {
		basin = basin_in;
		basin.enableBrakeMode(true);
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		basin.enableLimitSwitch(false, true);
		basin.setInverted(true);
		basin.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		isHoming = false;
	}

	public boolean getLimitSwitchState() {
		return basin.isRevLimitSwitchClosed();
	}

	public void home() {
		toVbs();
		if (!getLimitSwitchState()) {
			basin.set(-.1);
			isHoming = true;
		}
		else
		{
			isHoming = false;
		}
		
	}
	
	public boolean checkHome()
	{
		if (isHoming) {
				isHoming = !getLimitSwitchState();
				
			if (!isHoming) {
				System.out.println("You have reached the target (home).");
				basin.set(0);
				toEncPosition();
				try {
					Thread.sleep(100);
				} catch (InterruptedException e) {
					System.out.println("HOW?!?!?!??!");
					e.printStackTrace();
				}
				basin.setEncPosition(0);
			}

		}
		return isHoming;
	}
	
	public void moveUp()
	{
		toEncPosition();
		System.out.println("Moving Up");
		basin.enableControl();
		basin.set(convertInchesToRev(100));
	}
	
	public void moveDown()
	{	
		toEncPosition();
		System.out.println("Moving Down");
		basin.enableControl();
		basin.set(convertInchesToRev(-100));
	}
	
	public double getEncPosition()
	{
		return basin.getEncPosition();
	}
	
	public boolean isHoming()
	{
		return isHoming;
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
