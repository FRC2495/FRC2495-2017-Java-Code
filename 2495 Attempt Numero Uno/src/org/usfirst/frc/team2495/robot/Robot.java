package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	final String BaseBreak = "Baseline Breaker"; // Auton selection cases Dead Reckoning requires a start that is not gonna run into the steamship
	final String GearGrab = "Gear Grabber"; // Center gear placement
	final String GearGrabRight = "Gear Grabber Right"; // Right Gear Placement
	final String GearGrabLeft = "Gear Grabber Left";
	final String DankDump = "Dank Dumper"; //Dump Case 
	final String AutonDone = "Auton Done";
	final String Nothing = "Nothing";
	final String GearGrabCamera = "Gear Grabber With Vision";
	final String GearGrabRightCamera = "Gear Grabber Right With Vision";
	final String GearGrabLeftCamera = "Gear Grabber Left With Vision";

	String autoSelected;
	SendableChooser chooser;

	HMCamera camera;

	WPI_TalonSRX rearRight;
	WPI_TalonSRX frontRight;
	WPI_TalonSRX rearLeft;
	WPI_TalonSRX frontLeft;

	Joystick joyRight; // The Joysticks
	Joystick joyLeft;
	Joystick gamepad;

	ADXRS450_Gyro gyro; // gyro
	
	PowerDistributionPanel PDP = new PowerDistributionPanel(Ports.CAN.PDP);

	Drivetrain drivetrain; // DriveTrain object from the homemade class

	Timer time = new Timer(); // generic timer

	Take take;
	WPI_TalonSRX spin, climb;
	
	ControllerBase control;
	
	Compressor compressor; // the compressor's lifecycle needs to be the same as the robot
	
	boolean gearFlag;
	//boolean basinFlagUp = true;
	
	//Basin basinControl;
	double set;
	boolean hasGyroBeenManuallyCalibratedAtLeastOnce = false;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser = new SendableChooser();
		chooser.addDefault("Baseline Breaker", BaseBreak); // move forward
		chooser.addObject("Gear Grabber", GearGrab); // put the gear on the peg
		chooser.addObject("Gear Grabber Right Side", GearGrabRight);
		chooser.addObject("Gear Grabber Left Side", GearGrabLeft);
		chooser.addObject("Dank Dumper", DankDump); // go to the hopper and then
		chooser.addObject("Gear Grabber With Vision", GearGrabCamera);
		chooser.addObject("Gear Grabber Right Side With Vision", GearGrabRightCamera);
		chooser.addObject("Gera Grabber Left Side With Vision", GearGrabLeftCamera);
		chooser.addObject("Nothing", Nothing);

		rearRight = new WPI_TalonSRX(Ports.CAN.RIGHT_REAR); // The CANTalons
		frontRight = new WPI_TalonSRX(Ports.CAN.RIGHT_FRONT);
		rearLeft = new WPI_TalonSRX(Ports.CAN.LEFT_REAR);
		frontLeft = new WPI_TalonSRX(Ports.CAN.LEFT_FRONT);
		spin = new WPI_TalonSRX(Ports.CAN.SPIN);//intake spin motor
		climb = new WPI_TalonSRX(Ports.CAN.CLIMB); // climber motor 
		//basin = new WPI_TalonSRX(Ports.CAN.BASIN);

		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); // we want to instantiate before we pass to drivetrain		
		drivetrain = new Drivetrain(rearRight, frontRight, rearLeft, frontLeft, gyro, this);
		camera = new HMCamera("GRIP/myContoursReport");
 
		compressor = new Compressor();
		compressor.checkCompressor();

		take = new Take(spin, climb);

		joyLeft = new Joystick(Ports.USB.LEFT);
		joyRight = new Joystick(Ports.USB.RIGHT);
		gamepad = new Joystick(Ports.USB.GAMEPAD);
		

		control = new ControllerBase(gamepad, joyLeft, joyRight);
		
		//basinControl = new Basin(basin);
		gyro.calibrate(); 
		gyro.reset();
		//basinControl.home();
		set = .5;
		take.setGearPosition(Take.gearPosition.In);
	}

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 * 
	 * This is the intilization code for the autonomous, this code does:
	 * Sets the selecetd auton,
	 * Prints the selected Auton,
	 * Resets the timer,
	 * Puts the intake in the IN_UP / starting position,
	 * Resets Encoders.
	 */
	@Override
	public void autonomousInit() {
		autoSelected = (String) chooser.getSelected();
		System.out.println("Auto selected: " + autoSelected);
		time.reset();
		//take.setPosition(Take.Position.IN_UP);
		//drivetrain.resetEncoders();
	}

	/**
	 * This function is called periodically during autonomous
	 * 
	 * This code does:
	 * Update to Smart Dash,
	 * chooses what auton to run,
	 * updates smart dash again
	 */
	@Override
	public void autonomousPeriodic() {
		camera.acquireTargets(false);
		updateToSmartDash();
		
		switch (autoSelected) {
		case DankDump:
				// Put custom auto code here
			autoSelected = Nothing;
			break;
		case GearGrab:
			drivetrain.moveDistance(-75);
			drivetrain.waitMoveDistance();
			autoSelected = Nothing;
			break;
		case GearGrabRight:// TODO get measurements and fix this so its like GearGrab
			drivetrain.moveDistance(-85);
			drivetrain.waitMoveDistance();
			//drivetrain.moveDistanceAlongArc(135); 
			//drivetrain.waitMoveDistance(); // for moveDistanceAlongArc() only					
			drivetrain.angleSpotTurnUsingPidController(-60); // note: 120 degrees should be enough when using gyro
			drivetrain.waitAngleSpotTurnUsingPidController(); // for angleSpotTurnUsingPidController() only
			drivetrain.moveDistance(-40);
			drivetrain.waitMoveDistance();
			autoSelected = Nothing;
			break;
		case GearGrabLeft:// TODO get measurements and fix this so its like GearGrab
			drivetrain.moveDistance(-85);
			drivetrain.waitMoveDistance();
			//drivetrain.moveDistanceAlongArc(-135); 
			//drivetrain.waitMoveDistance(); // for moveDistanceAlongArc() only							
			drivetrain.angleSpotTurnUsingPidController(60);// note: -120 degrees should be enough when using gyro
			drivetrain.waitAngleSpotTurnUsingPidController(); // for angleSpotTurnUsingPidController() only
			drivetrain.moveDistance(-40);
			drivetrain.waitMoveDistance();
			autoSelected = Nothing;
			break;
		case BaseBreak:
			drivetrain.moveDistance(-95);
			drivetrain.waitMoveDistance();
			autoSelected = Nothing;
			break;
		case GearGrabCamera:
			drivetrain.moveDistance(-45);
			drivetrain.waitMoveDistance();
			visionBasedGearPlacingInAuton();
			autoSelected = Nothing;
			break;
		case GearGrabRightCamera:
			drivetrain.moveDistance(-85);
			drivetrain.waitMoveDistance();
			//drivetrain.moveDistanceAlongArc(135); 
			//drivetrain.waitMoveDistance(); // for moveDistanceAlongArc() only					
			drivetrain.angleSpotTurnUsingPidController(-60); // note: 120 degrees should be enough when using gyro
			drivetrain.waitAngleSpotTurnUsingPidController(); // for angleSpotTurnUsingPidController() only
			visionBasedGearPlacingInAuton();
			autoSelected = Nothing;
			break;
		case GearGrabLeftCamera:
			drivetrain.moveDistance(-85);
			drivetrain.waitMoveDistance();
			//drivetrain.moveDistanceAlongArc(-135); 
			//drivetrain.waitMoveDistance(); // for moveDistanceAlongArc() only					
			drivetrain.angleSpotTurnUsingPidController(60); // note: 120 degrees should be enough when using gyro
			drivetrain.waitAngleSpotTurnUsingPidController(); // for angleSpotTurnUsingPidController() only
			visionBasedGearPlacingInAuton();
			autoSelected = Nothing;
			break;
		case Nothing:
			//
			autoSelected = Nothing;
			break;
		}
		
		camera.acquireTargets(false);
		updateToSmartDash();
	}

	@Override
	public void teleopInit() {
		time.stop();
		time.reset();
		drivetrain.stop(); // very important!
	}

	/**
	 * This function is called periodically during operator control
	 *  
	 *  This code does:
	 *  checks for control updates,
	 *  checks to see if we are still moving after auton,
	 *  throws tankdrive controls to the joysticks,
	 *  sets intake/outtake position,
	 *  checks to see if a spin command is being given,
	 *  sets gear position,
	 *  sets basin position,
	 *  sets climber motor but only able to be done in the last 30 secs of match,
	 *  updates the smart dash,
	 *  debug commands for left joystick.
	 */
	@Override
	public void teleopPeriodic() {
		control.update();
		camera.acquireTargets(false);
		
		drivetrain.tripleCheckMoveDistance(); // checks if we are done moving if we were moving
		//drivetrain.checkMoveDistance(); // checks if we are done moving if we were moving
		drivetrain.triplecheckAngleSpotTurnUsingPidController(); // checks if we are done turning if we were turning 	
		//drivetrain.checkAngleSpotTurnUsingPidController(); // checks if we are done turning if we were turning // NOTE: consider double-checking instead
		//basinControl.checkHome();
		//basinControl.checkMove();
		
		// Tankdrive	
		
		drivetrain.joystickControl(joyRight, joyLeft, (control.getHeld(ControllerBase.Joysticks.LEFT_STICK,ControllerBase.JoystickButtons.BTN1) 
				                || control.getHeld(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN1))); //TODO calibrate joysticks
		
		
		// set in/outtake to position outdown = a, outup = b inup = x
//		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.A)) {
//			take.setPosition(Take.Position.OUT_DOWN);
//		} else if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.B)) {
//			take.setPosition(Take.Position.OUT_UP);
//		} else if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.X)) {
//			take.setPosition(Take.Position.IN_UP);
//		}

		// intake motor mapped to LB and RB
//		if(operator.getRawButton(ControllerBase.GamepadButtons.LB))
//		{
//			take.setSpin(1);
//		}
//		else if(operator.getRawButton(ControllerBase.GamepadButtons.RB))
//		{
//			take.setSpin(-1);
//		}
//		else
//		{
//			take.setSpin(0);
//		}
//		
		//gear bound to back
		if(control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.BACK))
		{
			
			if(gearFlag)
			{
				take.setGearPosition(Take.gearPosition.Out);
				gearFlag = false;
			}
			else
			{
				take.setGearPosition(Take.gearPosition.In);
				gearFlag = true;
			}
		}
		
		//basin bound to start
//		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.START)) {
//			System.out.println("Button Pushed");
//			if (basinFlagUp) {
//				basinControl.moveUp();
//				System.out.println("Should be Moving");
//				basinFlagUp = false;
//			} else {
//				basinControl.moveDown();
//				System.out.println("Should be Moving");
//				basinFlagUp = true;
//			}
//		}

		// Climber bound to y but can only be actived 2 minutes into the match
		
			
		if (control.getHeld(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.Y)) { 
			take.setClimb(set);
			set += .005;
		}
		else 
		{
			take.setClimb(0);		
		}
		
		//Stops the robot moving if pressed
		if(control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN3) || 
		   control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN3))
		{
			drivetrain.stop();
		}
		
		camera.acquireTargets(false);
		updateToSmartDash();
		
		// only enable these if debugging (or to have fun) 
		if(control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN2) ||
			control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN2))
		{
			angleSpotTurnUsingPidControllerTowardGearLift();
		}
		else if(control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN3) ||
			control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN3))
		{
			moveDistanceTowardGearLift();
	}
		else if(control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN4))
		{
			//drivetrain.moveDistanceAlongArc(-90);
			drivetrain.angleSpotTurnUsingPidController(-90);
		}
		else if(control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN5))
		{
			//drivetrain.moveDistanceAlongArc(+90);
			drivetrain.angleSpotTurnUsingPidController(+90);
		}
		else if(control.getPressedDown(ControllerBase.Joysticks.RIGHT_STICK, ControllerBase.JoystickButtons.BTN6))
		{
			drivetrain.moveDistance(50);
		}
		
//		if(control.getPressedDown(ControllerBase.Joysticks.LEFT_STICK, ControllerBase.JoystickButtons.BTN6))
//		{
//			basinControl.home();
//		}			
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	}

	@Override
	public void disabledInit() {
		SmartDashboard.putData("Alliterative Autoniomous Appointment", chooser);
	}
	/**
	 * This code does:
	 * updates controls
	 * allows an access to calibrate the gyro
	 * updates smart dash
	 */
	@Override
	public void disabledPeriodic() {	
		control.update();
		camera.acquireTargets(false);
		
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.A)) {
			gyro.calibrate();
			gyro.reset();
			hasGyroBeenManuallyCalibratedAtLeastOnce = true; // we flag that this was done
		}
		
		camera.acquireTargets(false);
		updateToSmartDash();
	}
	
	private void angleSpotTurnUsingPidControllerTowardGearLift() {
		drivetrain.angleSpotTurnUsingPidController(camera.getAngleToTurnToCenterOfTargets());
		/*drivetrain.angleSpotTurnUsingPidController(calculateProperTurnAngle(
				camera.getAngleToTurnToCenterOfTargets(),camera.getDistanceToCenterOfTargets()));*/
	}
	
	private void moveDistanceTowardGearLift() {
		final int OFFSET_CAMERA_GEARLIFT_INCHES = 10; // we need to leave some space between the camera and the targets
		final int MAX_DISTANCE_TO_GEARLIFT_INCHES = 120; // arbitrary very large distance
		
		// NOTE: if both targets cannot be seen in full at short distance
		// then we shall use getDistanceToCenterOfTargetsUsingHorizontalFov() to get the distance
		//double distanceReportedByCamera = camera.getDistanceToCenterOfTargetsUsingHorizontalFov();
		double distanceToTargetsReportedByCamera = camera.getDistanceToCenterOfTargets();
		
		if (distanceToTargetsReportedByCamera <= MAX_DISTANCE_TO_GEARLIFT_INCHES) {
			if (distanceToTargetsReportedByCamera >= OFFSET_CAMERA_GEARLIFT_INCHES) {
				drivetrain.moveDistance(-(distanceToTargetsReportedByCamera - OFFSET_CAMERA_GEARLIFT_INCHES)); // we need to move in reverse
			} else {
				System.out.println("Already at the gear lift!");
			}
		} else {
			System.out.println("Cannot move to infinity and beyond!");
		}		
	}
	
	private void visionBasedGearPlacingInAuton()
	{
		camera.acquireTargets(true);
		angleSpotTurnUsingPidControllerTowardGearLift();
		drivetrain.waitAngleSpotTurnUsingPidController();
		camera.acquireTargets(true);
		angleSpotTurnUsingPidControllerTowardGearLift();
		drivetrain.waitAngleSpotTurnUsingPidController();
		camera.acquireTargets(true);
		moveDistanceTowardGearLift();
		drivetrain.waitMoveDistance();
		take.setGearPosition(Take.gearPosition.Out);
		drivetrain.moveDistance(12);
		drivetrain.waitMoveDistance();
		take.setGearPosition(Take.gearPosition.In);
	}
	
	public void updateToSmartDash()
	{
		// Send Gyro val to Dashboard
        SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
        // send the gear status to dashboard
        SmartDashboard.putBoolean("Gear Good?", camera.checkForGearLift());
        SmartDashboard.putNumber("Right Value", drivetrain.getRightValue());
        SmartDashboard.putNumber("Left Value", drivetrain.getLeftValue());
        SmartDashboard.putNumber("Right Enc Value", drivetrain.getRightEncoderValue());
        SmartDashboard.putNumber("Left Enc Value", drivetrain.getLeftEncoderValue());
        SmartDashboard.putBoolean("isMoving?", drivetrain.isMoving());
        SmartDashboard.putBoolean("isTurning?", drivetrain.isTurning());
        SmartDashboard.putBoolean("isCompromised?", DriverStation.getInstance().isDisabled());
        SmartDashboard.putNumber("Distance to Target", camera.getDistanceToCenterOfTargets());
        SmartDashboard.putNumber("Angle to Target", camera.getAngleToTurnToCenterOfTargets());
        SmartDashboard.putNumber("Distance to Target Using Horizontal FOV", camera.getDistanceToCenterOfTargetsUsingHorizontalFov());
//        SmartDashboard.putBoolean("Basin Limit Switch", basinControl.getLimitSwitchState());
//        SmartDashboard.putNumber("Basin Position", basinControl.getPosition());
//        SmartDashboard.putNumber("Basin Enc Position", basinControl.getEncPosition());
//        SmartDashboard.putBoolean("Basin IsHoming?", basinControl.isHoming());
//        SmartDashboard.putBoolean("Basin IsMoving?", basinControl.isMoving());
//        SmartDashboard.putNumber("Basin Target", basinControl.getTarget());
//        SmartDashboard.putBoolean("Basin Has Been Homed?", basinControl.hasBeenHomed());
        SmartDashboard.putString("Auton selected", autoSelected!=null?autoSelected:"none");
        SmartDashboard.putBoolean("Gyro Manually Calibrated?",hasGyroBeenManuallyCalibratedAtLeastOnce);
        SmartDashboard.putNumber("PID Error", drivetrain.turnPidController.getError());
        SmartDashboard.putNumber("PID Motor Value", drivetrain.turnPidController.get());
        SmartDashboard.putBoolean("PID On Target", drivetrain.turnPidController.onTarget());
	}
	
	public double calculateProperTurnAngle(double cameraTurnAngle, double cameraHorizontalDist) {
		try {
			final double OFFSET_BETWEEN_CAMERA_AND_ROTATION_CENTER_INCHES = 6; // inches - might need adjustment
			double dist = cameraHorizontalDist * Math.cos(Math.toRadians(cameraTurnAngle));
			return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(cameraTurnAngle)) * dist
					/ (dist + OFFSET_BETWEEN_CAMERA_AND_ROTATION_CENTER_INCHES)));
		} catch (Exception e) {
			System.out.println("Exception in proper turn angle calculation" + e.toString());
			return 0;
		}
	}
}
