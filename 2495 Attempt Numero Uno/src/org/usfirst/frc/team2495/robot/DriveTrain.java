package org.usfirst.frc.team2495.robot;

import java.util.Calendar;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

// [GA] please add java doc explaining what the purpose of this class is
// Also please explain why it is extending the Robot class (or not make it extend the Robot class if not justified)
public class DriveTrain extends Robot {

	double Ltac, Rtac;
	final int TICK_THRESH = 50; // [GA] as you are using revolutions as unit of movement clean up unneeded stuff
	boolean isMoving;
	CANTalon RR, RF, LR, LF; // [GA] avoid using all uppercase variable names - reserve that for constants
	ADXRS450_Gyro gyro;
	int tickcount = 1024;
	double inchesPerTick = 4 * Math.PI / tickcount; // [GA] please define a symbolic constant for the diameter of the wheel as it is used in multiple places
	double ticksPerInch = tickcount / (4 * Math.PI);
	double revMulti = 4 * Math.PI;
	final int TIMEOUT_MS = 15000;

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro Gyro) {
		RR = rr; // sets the talons from the constructer to the talons used here
		RF = rf;
		LR = lr;
		LF = lf;

//		LF.setInverted(true); // inverts left side
//		LR.setInverted(true);

		RF.enableBrakeMode(true);// sets the talons on brake mode
		RR.enableBrakeMode(true);
		LR.enableBrakeMode(true);
		LF.enableBrakeMode(true);
		
		// [GA] simply noting that unit of distance will be revolutions when using position mode as side effect of using CtreMagEncoder
		RF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		LF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		RF.reverseOutput(true);
		LF.reverseSensor(true);

		LR.changeControlMode(TalonControlMode.Follower);
		RR.changeControlMode(TalonControlMode.Follower);
		LR.set(LF.getDeviceID());
		RR.set(RF.getDeviceID());
		//RF.setEncPosition(0);
		//LF.setEncPosition(0);
	}

	public void moveDistance(double dist) // moves the distance in inch given
	{
		RF.setPosition(0);
		LF.setPosition(0);
		Rtac = (dist / revMulti) + RF.getEncPosition();
		Ltac = (dist / revMulti) + LF.getEncPosition();
		System.out.println("Rtac,Ltac " + Rtac + " " + Ltac);
		toEnc(4);
		RF.enableControl();
		LF.enableControl();
		RF.set(Rtac);
		LF.set(Ltac);
	
			
		isMoving = true;
	}

	public boolean checkMoveDistance() {
		if (isMoving) {
			int Renc = (RF.getEncPosition());
			int Lenc = (LF.getEncPosition());
			//System.out.println("Renc,Lenc" + Renc + " " + Lenc);
/*			isMoving = !(Renc > Rtac - TICK_THRESH && Renc < Rtac + TICK_THRESH && 
						Lenc > Ltac - TICK_THRESH && Lenc < Ltac + TICK_THRESH);*/
		
			isMoving = Renc < Rtac && Lenc < Ltac; // [GA] would that work if you are going backwards?
			if(!isMoving)
			{
				System.out.println("You have reached the target.");
				stop();
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
				System.out.println("You went over the time limit");
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

	// [GA] please explain how this method should be used (including preconditions and postconditions)
	public void angleSpotTurn(int angle) // turns on the spot to the specified
											// angle
	{
		toVbs();
		double current = gyro.getAngle();
		angle += current;
		if (angle > gyro.getAngle() + 2) {
			LF.set(.5);
			RF.set(-.5);

		} else if (angle < gyro.getAngle() - 2) {
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

	// [GA] it would be better to call this method toEncPos() as encoders can also be used for other modes (e.g. speed)
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
	}

	public void toVbs() // sets talons to voltage control
	{
		RF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		LF.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		// RR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		// LR.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}

	// [GA] no issue when the joysticks are resting? 
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
		return (int) (RF.getPosition() * 1);//inchesPerTick);
	}

	public int getLEncVal() {
		return (int) (LF.getPosition() * 1);//inchesPerTick);
	}

	public boolean getIsMoving() {
		return isMoving;
	}

}