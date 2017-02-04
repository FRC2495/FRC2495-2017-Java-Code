package org.usfirst.frc.team2495.robot;

import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;


public class DriveTrain extends Robot {
	
	int Renc, Lenc;
	CANTalon RR,RF,LR,LF;
	ADXRS450_Gyro gyro;

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro Gyro) {
		CANTalon RR = new CANTalon(rr.getDeviceID()); // sets the talons from the constructer to the talons used here
		CANTalon RF = new CANTalon(rf.getDeviceID());
		CANTalon LR = new CANTalon(lr.getDeviceID());
		CANTalon LF = new CANTalon(lf.getDeviceID());
		
		RR.changeControlMode(CANTalon.TalonControlMode.Follower); // Sets rear talons to follow the front ones
		RR.set(RF.getDeviceID());
		LR.changeControlMode(CANTalon.TalonControlMode.Follower);
		LR.set(LF.getDeviceID());
		LF.setInverted(true); // inverts left side
		
		RF.enableBrakeMode(true);//sets the talons on brake mode
		RR.enableBrakeMode(true);
		LR.enableBrakeMode(true);
		LF.enableBrakeMode(true);
		
		
	}
	
	public void moveDistance (double dist) //moves the distance in feet given 
	{
		int tickcount = 1024;
		double inchtick =  tickcount / 4 * Math.PI;
		double feettick = 12 * inchtick;
		Renc = RF.getEncPosition();
		Lenc = LF.getEncPosition();
		toEnc();
		RF.setEncPosition((int) ((int)feettick * dist + Renc));
		LF.setEncPosition((int) ((int)feettick * dist + Lenc));
		
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
		if(angle > gyro.getAngle())
		{
			LF.set(.5);
			RF.set(-.5);
		}
		else if (angle < gyro.getAngle())
		{
			LF.set(-.5);
			RF.set(.5);
		}
		else
		{
			LF.set(0);
			RF.set(0);
		}
	}
	
	public void toEnc() // sets the talons to encoder control
	{
		RF.changeControlMode(CANTalon.TalonControlMode.Position);
		LF.changeControlMode(CANTalon.TalonControlMode.Position);
	}
	
	public void toVbs() //sets talons to voltage control
	{
		RF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		LF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	public void joystickControl(Joystick r, Joystick l) // sets talons to joystick control
	{
		toVbs();
		RF.set(r.getY());
		LF.set(l.getY());
	}

}