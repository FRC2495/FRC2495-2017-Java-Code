package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


public class DriveTrain extends Robot {
	
	int Renc, Lenc;
	CANTalon RR,RF,LR,LF;
	ADXRS450_Gyro gyro;
	int tickcount = 1024;
	double inchesPerTick = 4 * Math.PI / tickcount;
	double ticksPerInch = tickcount / (4 * Math.PI);

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro Gyro) {
		RR = rr; // sets the talons from the constructer to the talons used here
		RF = rf;
		LR = lr;
		LF = lf;
		
		LF.setInverted(true); // inverts left side
		LR.setInverted(true);
		
		RF.enableBrakeMode(true);//sets the talons on brake mode
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
	 
	public void moveDistance (double dist) //moves the distance in inch given 
	{
		Renc = RF.getEncPosition();
		Lenc = LF.getEncPosition();
		toEnc(6);
		RF.set((int) (ticksPerInch * dist) + Renc);
		LF.set((int) (ticksPerInch * dist) + Lenc);

	}
	
	public void moveForward()
	{
		toVbs();
		RF.set(.5);
		LF.set(.5);
	}
	
	public void stop ()
	{
		toVbs();
		RF.set(0);
		LF.set(0);

	}
	public void angleSpotTurn (int angle) // turns on the spot to the specified angle
	{
		toVbs();
		double current = gyro.getAngle();
		angle += current;
		if(angle > gyro.getAngle())
		{
			LF.set(.5);
			RF.set(-.5);

		}
		else if (angle < gyro.getAngle())
		{
			LF.set(-.5);
			RF.set(.5);
//			LR.set(-.5);
//			RR.set(.5);
		}
		else
		{
			LF.set(0);
			RF.set(0);
//			LR.set(0);
//			RR.set(0);
		}
	}
	
	public void toEnc(int forward) // sets the talons to encoder control
	{
		RF.enableControl();
		LF.enableControl();
		RF.setPID(0.4, 0, 0);
		LF.setPID(0.4, 0, 0);
		RF.changeControlMode(CANTalon.TalonControlMode.Position);
		LF.changeControlMode(CANTalon.TalonControlMode.Position);
		RF.configPeakOutputVoltage(12, -12);
		LF.configPeakOutputVoltage(12, -12);
		RF.configNominalOutputVoltage(forward,0);
		LF.configNominalOutputVoltage(forward,0);
//		RR.changeControlMode(CANTalon.TalonControlMode.Position);
//		LR.changeControlMode(CANTalon.TalonControlMode.Position);
	}
	
	public void toVbs() //sets talons to voltage control
	{
		RF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		LF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//		RR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
//		LR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	public void joystickControl(Joystick r, Joystick l) // sets talons to joystick control
	{
		toVbs();
		RF.set(r.getY());
		LF.set(l.getY());
//		RR.set(r.getY());
//		LR.set(l.getY());
	}
	
	public int getREncVal()
	{
		return (int)(RF.getEncPosition() * inchesPerTick);
	}
	public int getLEncVal()
	{
		return (int)(LF.getEncPosition() * inchesPerTick);
	}

}