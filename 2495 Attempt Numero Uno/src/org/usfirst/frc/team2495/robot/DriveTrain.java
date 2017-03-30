package org.usfirst.frc.team2495.robot;

import java.util.Calendar;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.ctre.CANTalon.TalonControlMode;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;

// [GA] please add java doc explaining what the purpose of this class is [SP] no idea how to do javadoc
public class DriveTrain implements PIDOutput {

	double Ltac, Rtac;
	boolean isMoving, isTurning;
	boolean wasOnTarget;
	CANTalon RR, RF, LR, LF; // [GA] avoid using all uppercase variable names -
								// reserve that for constants
	ADXRS450_Gyro gyro;
	
	static final double REV_MULTI = 4 * Math.PI;
	static final int TIMEOUT_MS = 15000;
	static final double REV_THRESH = .125;
	static final int RADIUS_DRIVEVETRAIN_INCHES = 13;
	static final double MOVING_VOLTAGE_VOLTS = 4.0;
	
	// NOTE: it might make sense to decrease the PID controller period to 0.02 sec (which is the period used by the main loop)
	static final double TURN_PID_CONTROLLER_PERIOD_SECONDS = PIDController.kDefaultPeriod ; // 0.05 sec = 50 ms 
	Robot robot;
	
	PIDController turnPidController;

	public DriveTrain(CANTalon rr, CANTalon rf, CANTalon lr, CANTalon lf, ADXRS450_Gyro gyro_in, Robot robot_in) {
		RR = rr; // sets the talons from the constructer to the talons used here
		RF = rf;
		LR = lr;
		LF = lf;
		robot = robot_in;
		gyro = gyro_in;	

		RF.enableBrakeMode(true);// sets the talons on brake mode
		RR.enableBrakeMode(true);
		LR.enableBrakeMode(true);
		LF.enableBrakeMode(true);

		// [GA] simply noting that unit of distance will be revolutions when
		// using position mode as side effect of using CtreMagEncoder
		RF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		LF.setFeedbackDevice(FeedbackDevice.CtreMagEncoder_Relative);
		
		RF.reverseOutput(true); // we need to reverse output
		LF.reverseOutput(false); // no need to reverse output
		
		LF.reverseSensor(true); // lf encoder reversed
		LF.setInverted(true); // lf motor reversed

		LR.changeControlMode(TalonControlMode.Follower);
		RR.changeControlMode(TalonControlMode.Follower);
		LR.set(LF.getDeviceID());
		RR.set(RF.getDeviceID());
		// RF.setEncPosition(0);
		// LF.setEncPosition(0);
		
    	//creates a PID controller
		turnPidController = new PIDController(0.17, 0.0002, 0.0, gyro, this, TURN_PID_CONTROLLER_PERIOD_SECONDS);
    	turnPidController.setContinuous(true); // because -180 degrees is the same as 180 degrees
    	turnPidController.setAbsoluteTolerance(1); // 1 degree error tolerated
    	
    	//NOTE: setToleranceBuffer should be set to 1 if trying doublecheckAngleSpotTurnUsingPidController()
    	// as doublecheckAngleSpotTurnUsingPidController() already takes two measurements.
    	// Using a tolerance buffer of 3 with single checkAngleSpotTurnUsingPidController() could
    	// also be an option, but that would add another 50 ms to the turn
    	// (or whatever period is set if a non-default period is used)
    	turnPidController.setToleranceBuffer(2); // indicates that we want two measurements before accepting that we are on target    	
    	turnPidController.setInputRange(-180, 180); // valid input range 
    	turnPidController.setOutputRange(-.5, .5); // output range NOTE: might need to change signs
	}

	// this method needs to be paired with checkAngleSpotTurnUsingPidController()
	public void angleSpotTurnUsingPidController(double angle) {
		toVbs(); // switches to percentage vbus
		stop(); // resets state
		
		gyro.reset(); // resets to zero for now
		//double current = gyro.getAngle();
		double heading = angle; //+ current; // calculates new heading
		
		turnPidController.setSetpoint(heading); // sets the heading
		turnPidController.enable(); // begins running
		
		isTurning = true;
		wasOnTarget = false; // resets the flag 
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
	
	// NOTICE: this experimental method should only be tried with a tolerance buffer of one 
	// The difference between doublecheckAngleSpotTurnUsingPidController and using a tolerance buffer of two
	// is that this method checks that we were on target twice in a row rather than that the average was on target
	// twice in a row. The average could conceivably be be on target even if we were never on target, 
	// so while this method is somewhat ugly it might still work better than using a tolerance buffer of two.
	// Using a tolerance buffer of three with single checkAngleSpotTurnUsingPidController() is also an option.
	// Also it might be worth reducing TURN_PID_CONTROLLER_PERIOD_SECONDS to 0.02 sec to tighten the control loop.
	public boolean doublecheckAngleSpotTurnUsingPidController() {
		if (isTurning) {
			boolean isOnTarget = turnPidController.onTarget();
			
			if (isOnTarget) { // if we are on target in this iteration 
				if (wasOnTarget) { // and if we were already on target in the last iteration
					isTurning = false; // we consider that we really reached the target and we stop turning
				} else {					
					wasOnTarget = true; // we note that we are on target but we don't stop yet
					System.out.println("You might have reached the target (turning). Please check again.");
					
					try {
						// sleeps the period of the turnPidController so that onTarget() will return a fresh value next time
						// this is deemed acceptable because the delay is very short and only triggered when we are allegedly on target
						Thread.sleep((long)(TURN_PID_CONTROLLER_PERIOD_SECONDS*1000));
					} catch (InterruptedException e) {
						e.printStackTrace();
					}
				}
			} else { // if we are not on target in this iteration
				if (wasOnTarget) { // even though we were on target in the last iteration
					wasOnTarget = false; // we discard the fact that we could have felt we were on target during the last iteration
					System.out.println("Double-check failed.");
				} else {
					// we are definitely turning
				}
			}
			
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

		//while (doublecheckAngleSpotTurnUsingPidController()) { 		
		while (checkAngleSpotTurnUsingPidController()) { // NOTE: consider double-checking instead
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
			
			robot.updateToSmartDash();
		}		
		stop();
	}

	// this method needs to be paired with checkMoveDistance()
	public void moveDistance(double dist) // moves the distance in inch given
	{
		toVbs();
		RF.setPosition(0);
		LF.setPosition(0);	
		//resetEncoders();
		toEnc(MOVING_VOLTAGE_VOLTS);
		Rtac = (dist / REV_MULTI);
		Ltac = (dist / REV_MULTI);
		System.out.println("Rtac,Ltac " + Rtac + " " + Ltac);
		RF.enableControl();
		LF.enableControl();
		RF.set(Rtac);
		LF.set(Ltac);

		isMoving = true;
	}

	public boolean checkMoveDistance() {
		if (isMoving) {
			double Renc = RF.getPosition(); 
			double Lenc = LF.getPosition(); 
			// System.out.println("Renc,Lenc" + Renc + " " + Lenc);
			isMoving = !(Renc > Rtac - REV_THRESH && Renc < Rtac + REV_THRESH && Lenc > Ltac - REV_THRESH
					&& Lenc < Ltac + REV_THRESH);

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
			
			robot.updateToSmartDash();
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
		toVbs();
		RF.setPosition(0);
		LF.setPosition(0);	
		//resetEncoders();

		Rtac = (rdist / REV_MULTI);
		Ltac = (ldist / REV_MULTI);
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
				&& DriverStation.getInstance().isAutonomous()) { // NOTE: in teleop this condition is false, so the routine will exit right away
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
	public void joystickControl(Joystick r, Joystick l, boolean held) // sets talons to
														// joystick control
	{
		if (!isMoving && !isTurning) // if we are already doing a move or turn we don't take over
		{
			if(!held)
			{
				toVbs();
				RF.set(r.getY() * (5.0/8));
				LF.set(l.getY() * (5.0/8));
			}
			else
			{
				toVbs();
				RF.set(r.getY());
				LF.set(l.getY());
			}
		}
	}

	public int getREncVal() {
		return (int) (RF.getEncPosition());
	}

	public int getLEncVal() {
		return (int) (LF.getEncPosition());
	}

	public int getRVal() {
		return (int) (RF.getPosition());
	}

	public int getLVal() {
		return (int) (LF.getPosition());
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
		RF.set(+output);
		LF.set(-output);		
	}
	
	public void resetEncoders() {
		toVbs();
		RF.setEncPosition(0);
		LF.setEncPosition(0);
	}
}