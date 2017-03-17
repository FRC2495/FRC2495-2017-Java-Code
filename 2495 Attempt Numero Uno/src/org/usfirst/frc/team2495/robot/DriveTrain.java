package org.usfirst.frc.team2495.robot;

import java.util.Calendar;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

// [GA] please add java doc explaining what the purpose of this class is [SP] no idea how to do javadoc
public class DriveTrain implements PIDOutput {

	double Ltac, Rtac;
	boolean isMoving, isTurning;
	CANTalon RR, RF, LR, LF; // [GA] avoid using all uppercase variable names -
								// reserve that for constants
	ADXRS450_Gyro gyro;
	
	static final double REV_MULTI = 4 * Math.PI;
	static final int TIMEOUT_MS = 15000;
	static final double REV_THRESH = .125;
	static final int RADIUS_DRIVEVETRAIN_INCHES = 13;
	static final double MOVING_VOLTAGE_VOLTS = 4.0;
	
	PIDController turnPidController;

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro Gyro) {
		RR = rr; // sets the talons from the constructer to the talons used here
		RF = rf;
		LR = lr;
		LF = lf;

		// LF.setInverted(true); // inverts left side
		// LR.setInverted(true);

		RF.enableBrakeMode(true);// sets the talons on brake mode
		RR.enableBrakeMode(true);
		LR.enableBrakeMode(true);
		LF.enableBrakeMode(true);

		// [GA] simply noting that unit of distance will be revolutions when
		// using position mode as side effect of using CtreMagEncoder
		RF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		LF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		RF.reverseOutput(true);
		LF.reverseSensor(true);

		LR.changeControlMode(TalonControlMode.Follower);
		RR.changeControlMode(TalonControlMode.Follower);
		LR.set(LF.getDeviceID());
		RR.set(RF.getDeviceID());
		// RF.setEncPosition(0);
		// LF.setEncPosition(0);
		
    	//creates a PID controller
		turnPidController = new PIDController(0.17, 0.0002, 0.0, Gyro, this);
    	turnPidController.setContinuous(true); // because -180 degrees is the same as 180 degrees
    	turnPidController.setAbsoluteTolerance(1); // 1 degree error tolerated
    	turnPidController.setInputRange(-180, 180); // valid input range 
    	turnPidController.setOutputRange(-.5, .5); // output range NOTE: might need to change signs
	}

	// this method needs to be paired with checkAngleSpotTurnUsingPidController()
	public void angleSpotTurnUsingPidController(int angle) {
		toVbs(); // switches to percentage vbus
		stop(); // resets state
		
		double current = gyro.getAngle();
		double heading = angle + current; // calculates new heading
		
		turnPidController.setSetpoint(heading); // sets the heading
		turnPidController.enable(); // begins running
		
		isTurning = true;
	}
	
	public boolean checkAngleSpotTurnUsingPidController() {
		if (isTurning) {
			isTurning = !turnPidController.onTarget();
			
			if (!isTurning) {
				System.out.println("You have reached the target (turning).");
				stop();
				toVbs();
			}

		}
		return isTurning;
	}
	
	// do not use in teleop - for auton only
	public void waitAngleSpotTurnUsingPidController() {
		long start = Calendar.getInstance().getTimeInMillis();
		
		while (checkAngleSpotTurnUsingPidController()) {
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= TIMEOUT_MS) {
				System.out.println("You went over the time limit (turning)");
				stop();
				break;
			}

			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}		
		stop();
	}

	// this method needs to be paired with checkMoveDistance()
	public void moveDistance(double dist) // moves the distance in inch given
	{
		RF.setPosition(0);
		LF.setPosition(0);
		Rtac = (dist / REV_MULTI) + RF.getEncPosition();
		Ltac = (dist / REV_MULTI) + LF.getEncPosition();
		System.out.println("Rtac,Ltac " + Rtac + " " + Ltac);
		toEnc(MOVING_VOLTAGE_VOLTS);
		RF.enableControl();
		LF.enableControl();
		RF.set(Rtac);
		LF.set(Ltac);

		isMoving = true;
	}

	public boolean checkMoveDistance() {
		if (isMoving) {
			int Renc = (RF.getEncPosition()); // FIXME getPosition() is probably what we need here
			int Lenc = (LF.getEncPosition()); // FIXME getPosition() is probably what we need here
			// System.out.println("Renc,Lenc" + Renc + " " + Lenc);
			isMoving = !(Renc > Rtac - REV_THRESH && Renc < Rtac + REV_THRESH && Lenc > Ltac - REV_THRESH
					&& Lenc < Ltac + REV_THRESH);

			// isMoving = Renc < Rtac && Lenc < Ltac; // [GA] would that work if
			// you are going backwards?
			if (!isMoving) {
				System.out.println("You have reached the target (moving).");
				stop();
				toVbs();
			}
		}
		return isMoving;
	}

	// do not use in teleop - for auton only
	public void waitMoveDistance() {
		long start = Calendar.getInstance().getTimeInMillis();
		
		while (checkMoveDistance()) {
			if (!DriverStation.getInstance().isAutonomous()
					|| Calendar.getInstance().getTimeInMillis() - start >= TIMEOUT_MS) {
				System.out.println("You went over the time limit (moving)");
				stop();
				break;
			}
			
			try {
				Thread.sleep(20); // sleeps a little
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
		}
	}

	private double arclength(int angle) // returns the inches needed to be moved
										// to turn the specified angle
	{
		return Math.toRadians(angle) * RADIUS_DRIVEVETRAIN_INCHES;
	}

	// this method needs to be paired with checkMoveDistance()
	public void moveDistanceAlongArc(int angle) {
		double dist = arclength(angle);
		double ldist, rdist;

		ldist = dist;
		rdist = -dist;

		RF.setPosition(0);
		LF.setPosition(0);
		Rtac = (rdist / REV_MULTI) + RF.getEncPosition();
		Ltac = (ldist / REV_MULTI) + LF.getEncPosition();
		System.out.println("Rtac,Ltac " + Rtac + " " + Ltac);
		toEnc(MOVING_VOLTAGE_VOLTS);
		RF.enableControl();
		LF.enableControl();
		RF.set(Rtac);
		LF.set(Ltac);

		isMoving = true;
	}

	public void moveForward() {
		toVbs();
		RF.set(.5);
		LF.set(.5);
	}

	public void stop() {
		turnPidController.disable(); // exits PID loop
		toVbs();
		RF.set(0);
		LF.set(0);
		isMoving = false;
		isTurning = false;
	}

	// this method is blocking (will only return once the turn is completed if ever) - do not use in teleop
	public void angleSpotTurn(int angle) // turns on the spot to the specified
											// angle clockwise is positive
											// movement
	{
		toVbs();
		stop();
		double current = gyro.getAngle();
		double heading = angle + current;
		while ((heading > gyro.getAngle() + 2 || heading < gyro.getAngle() - 2)
				&& DriverStation.getInstance().isAutonomous()) {
			if (heading > gyro.getAngle() + 2) {
				LF.set(.5);
				RF.set(-.5);

			} else if (heading < gyro.getAngle() - 2) {
				LF.set(-.5);
				RF.set(.5);
			}
		}
		stop();
	}

	// [GA] it would be better to call this method toEncPos() as encoders can
	// also be used for other modes (e.g. speed)
	public void toEnc(double forward) // sets the talons to encoder control
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
	}

	// [GA] no issue when the joysticks are resting? [SP] yeah its fine when its
	// resting
	public void joystickControl(Joystick r, Joystick l) // sets talons to
														// joystick control
	{
		if (!isMoving && !isTurning) // if we are already doing a move or turn we don't take over
		{
			toVbs();
			RF.set(r.getY());
			LF.set(-l.getY());
			// RR.set(r.getY());
			// LR.set(l.getY());
		}
	}

	public int getREncVal() {
		return (int) (RF.getPosition() * 1);
	}

	public int getLEncVal() {
		return (int) (LF.getPosition() * 1);
	}

	public boolean isMoving() {
		return isMoving;
	}
	
	public boolean isTurning(){
		return isTurning;
	}

	@Override
	public void pidWrite(double output) {
		toVbs();
		RF.set(-output); //NOTE: sign might need to be reverted
		LF.set(output); //NOTE: sign might need to be reverted		
	}

}