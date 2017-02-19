package org.usfirst.frc.team2495.robot;

import java.util.Calendar;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

public class DriveTrain extends Robot {

	int Ltac, Rtac;
	final int TICK_THRESH = 50;
	boolean isMoving;
	CANTalon RR, RF, LR, LF;
	ADXRS450_Gyro gyro;
	int tickcount = 1024;
	double inchesPerTick = 4 * Math.PI / tickcount;
	double ticksPerInch = tickcount / (4 * Math.PI);
	final int TIMEOUT_MS = 15000;

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro Gyro) {
		RR = rr; // sets the talons from the constructer to the talons used here
		RF = rf;
		LR = lr;
		LF = lf;

		LF.setInverted(true); // inverts left side
		LR.setInverted(true);

		RF.enableBrakeMode(true);// sets the talons on brake mode
		RR.enableBrakeMode(true);
		LR.enableBrakeMode(true);
		LF.enableBrakeMode(true);

		LR.changeControlMode(TalonControlMode.Follower);
		RR.changeControlMode(TalonControlMode.Follower);
		LR.set(LF.getDeviceID());
		RR.set(RF.getDeviceID());
		RF.setEncPosition(0);
		LF.setEncPosition(0);
	}

	public void moveDistance(double dist) // moves the distance in inch given
	{
		Rtac = (int) (ticksPerInch * dist);
		Ltac = (int) (ticksPerInch * dist);
		toEnc(6);
		RF.setEncPosition(0);
		LF.setEncPosition(0);
		RF.set(Rtac);
		LF.set(Ltac);
		RF.enableControl();
		LF.enableControl();
		isMoving = true;
	}

	public boolean checkMoveDistance() {
		if (isMoving) {
			int Renc = RF.getEncPosition();
			int Lenc = LF.getEncPosition();
			isMoving = (Renc > Rtac - TICK_THRESH && Renc < Rtac + TICK_THRESH && 
						Lenc > Ltac - TICK_THRESH && Lenc < Ltac + TICK_THRESH);
			if(!isMoving)
			{
				toVbs();
			}
		}
		return isMoving;
	}
	
	public void waitMove()
	{
		long start = Calendar.getInstance().getTimeInMillis();
		while(checkMoveDistance())
		{
			if(!DriverStation.getInstance().isAutonomous() || Calendar.getInstance().getTimeInMillis() - start >= TIMEOUT_MS)
			{
				stop();
				break;
			}
		}
	}

	public void moveForward() {
		toVbs();
		RF.set(.5);
		LF.set(.5);
	}

	public void stop() {
		toVbs();
		RF.set(0);
		LF.set(0);
		isMoving = false;

	}

	public void angleSpotTurn(int angle) // turns on the spot to the specified
											// angle
	{
		toVbs();
		double current = gyro.getAngle();
		angle += current;
		if (angle > gyro.getAngle()) {
			LF.set(.5);
			RF.set(-.5);

		} else if (angle < gyro.getAngle()) {
			LF.set(-.5);
			RF.set(.5);
			// LR.set(-.5);
			// RR.set(.5);
		} else {
			LF.set(0);
			RF.set(0);
			// LR.set(0);
			// RR.set(0);
		}
	}

	public void toEnc(int forward) // sets the talons to encoder control
	{
		RF.setPID(0.4, 0, 0);
		LF.setPID(0.4, 0, 0);
		RF.changeControlMode(CANTalon.TalonControlMode.Position);
		LF.changeControlMode(CANTalon.TalonControlMode.Position);
		RF.configPeakOutputVoltage(forward, -forward);
		LF.configPeakOutputVoltage(forward, -forward);
		RF.configNominalOutputVoltage(0, 0);
		LF.configNominalOutputVoltage(0, 0);
		// RR.changeControlMode(CANTalon.TalonControlMode.Position);
		// LR.changeControlMode(CANTalon.TalonControlMode.Position);
	}

	public void toVbs() // sets talons to voltage control
	{
		RF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		LF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		// RR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		// LR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	public void joystickControl(Joystick r, Joystick l) // sets talons to
														// joystick control
	{
		toVbs();
		RF.set(r.getY());
		LF.set(l.getY());
		// RR.set(r.getY());
		// LR.set(l.getY());
	}

	public int getREncVal() {
		return (int) (RF.getEncPosition() * inchesPerTick);
	}

	public int getLEncVal() {
		return (int) (LF.getEncPosition() * inchesPerTick);
	}

	public boolean getIsMoving() {
		return isMoving;
	}

}