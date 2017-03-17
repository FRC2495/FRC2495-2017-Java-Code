package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;

public class Basin {
	CANTalon basin;
	static final double SCREW_PITCH_INCHES_PER_REV = .75; 
	static final int LENGTH_OF_SCREW_INCHES = 13;
	boolean isHoming, isMoving;
	final double REV_THRESH = .125;
	double tac;

	public Basin(CANTalon basin_in) {
		basin = basin_in;
		basin.enableBrakeMode(true);
		basin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		basin.enableLimitSwitch(false, true);
		basin.setInverted(false);
		basin.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		isHoming = false;
		isMoving = false;
	}

	public boolean getLimitSwitchState() {
		return basin.isRevLimitSwitchClosed();
	}

	public void home() {
		toVbs();
		if (!getLimitSwitchState()) {
			basin.set(.1);
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
				toEncPosition(4);
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
	
	public boolean checkMove()
	{
		if (isMoving) {
				int enc = basin.getEncPosition();

				isMoving = !(enc > tac - REV_THRESH && enc < tac + REV_THRESH);
				
			if (!isMoving) {
				System.out.println("You have reached the target (basin moving).");
				toVbs();
				basin.set(0);
			}

		}
		return isMoving;
	}
	
	public void moveUp()
	{
		toEncPosition(4);
		System.out.println("Moving Up");
		basin.setSafetyEnabled(false);
		basin.enableControl();
		tac = -convertInchesToRev(100);
		basin.set(tac);
		isMoving = true;
	}
	
	public void moveDown()
	{	
		toEncPosition(4);
		System.out.println("Moving Down");
		basin.enableControl();
		tac = 0;
		basin.set(tac);
		isMoving = true;
	}
	
	public double getEncPosition()
	{
		return basin.getEncPosition();
	}
	
	public boolean isHoming()
	{
		return isHoming;
	}
	
	public boolean isMoving()
	{
		return isMoving;
	}
	
	private double convertInchesToRev (double inches)
	{
		return inches / SCREW_PITCH_INCHES_PER_REV;
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

}
