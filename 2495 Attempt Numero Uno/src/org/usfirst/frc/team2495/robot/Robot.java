package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import java.util.ResourceBundle.Control;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	
	//[GA] please explain what the various auton modes are really doing
	// (and list the assumptions made regarding the starting positions)
	final String BaseBreak = "Baseline Breaker"; // Auton selection cases Dead Reckoning
	final String GearGrab = "Gear Grabber"; // Center gear placement
	final String GearGrabRight = "Gear Grabber Right"; // Right Gear Placement
	final String DankDump = "Dank Dumper"; //Dump Case 
	final String AutonDone = "Auton Done";

	String autoSelected;
	SendableChooser chooser;

	HMCamera camera;

	CANTalon RR;
	CANTalon RF;
	CANTalon LR;
	CANTalon LF;
	// CANTalon intake;

	Joystick right; // The Joysticks
	Joystick left;
	Joystick operator;

	ADXRS450_Gyro gyro; // gyro
	
	PowerDistributionPanel PDP = new PowerDistributionPanel(/*Ports.CAN.PCM*/6); // PDP FIXME SHOULD THIS BE 6 OR 7???

	DriveTrain drivetrain; // DriveTrain object from the homemade class

	Timer time = new Timer(); // generic timer

	double centerx = 0.0; // center of the x axis

	Take take;
	CANTalon spin, climb;
	ControllerBase control;
	
	boolean gearFlag, basinFlag;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser = new SendableChooser();
		chooser.addDefault("Baseline Breaker", BaseBreak); // move forward
		chooser.addObject("Gear Grabber", GearGrab); // put the gear on the peg
		chooser.addObject("Gear Grabber Left Side (WIP DO NOT PICK)", GearGrabRight);
		chooser.addObject("Dank Dumper", DankDump); // go to the hopper and then

		//[GA] consider centralizing all the device numbers/ports in one location (e.g. Ports.java)
		RR = new CANTalon(Ports.CAN.RIGHT_REAR); // The CANTalons
		RF = new CANTalon(Ports.CAN.RIGHT_FRONT);
		LR = new CANTalon(Ports.CAN.LEFT_REAR);
		LF = new CANTalon(Ports.CAN.LEFT_FRONT);
		spin = new CANTalon(Ports.CAN.SPIN);//intake spin motor
		climb = new CANTalon(Ports.CAN.CLIMB); // climber motor 

		drivetrain = new DriveTrain(RR, RF, LR, LF, gyro);
		camera = new HMCamera("myContoursReport");
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0); 

		Compressor compressor = new Compressor();
		compressor.checkCompressor();

		take = new Take(spin, climb);

		left = new Joystick(Ports.USB.LEFT);
		right = new Joystick(Ports.USB.RIGHT);
		operator = new Joystick(Ports.USB.GAMEPAD);

		control = new ControllerBase(operator, left, right);
		
		gyro.calibrate(); 
		gyro.reset();
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
	 */
	@Override
	public void autonomousInit() {
		autoSelected = (String) chooser.getSelected();
		// autoSelected = SmartDashboard.getString("Auto Selector",
		// defaultAuto);
		System.out.println("Auto selected: " + autoSelected);
		//gyro.calibrate(); you do NOT want to waste time calibrating at the beginning of a match 
		time.reset();
		take.setPosition(Take.Position.IN_UP);
		RF.setEncPosition(0);
		LF.setEncPosition(0);
	}

	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		updateToSmartDash();
		
		switch (autoSelected) {
		case DankDump:
			// Put custom auto code here
			break;
		case GearGrab: { 
				drivetrain.moveDistance(75);
				drivetrain.waitMove();
				drivetrain.angleSpotTurn(180);
				drivetrain.waitMove();
				drivetrain.moveDistance(-20);
				drivetrain.waitMove();
				take.setGearPosition(Take.gearPosition.Out);
			
		}
			break;
		case GearGrabRight: { // TODO get measurements and fix this so its like GearGrab
			drivetrain.moveDistance(120);
			drivetrain.angleSpotTurn(70);
			// if (camera.checkForGear()) {
			// if (camera.getHeight()[0] == 200) {
			// drivetrain.moveForward();
			// } else {
			// drivetrain.stop();
			// }
			// }
			drivetrain.moveDistance(20);
			take.setGearPosition(Take.gearPosition.Out);
			drivetrain.angleSpotTurn(240);
			drivetrain.moveDistance(20);
			// while intaking
			drivetrain.angleSpotTurn(90);
			drivetrain.moveDistance(120);
			// outtake
		}
			break;
		case BaseBreak: {
			RF.setPosition(0);
			LF.setPosition(0);
			if (!drivetrain.getIsMoving()) {
				drivetrain.moveDistance(95);
			}
		}
			break;
		}
		
		updateToSmartDash();
		//SmartDashboard.putBoolean("isCompromised?", DriverStation.getInstance().isDisabled());
	}

	@Override
	public void teleopInit() {
		time.stop();
		time.reset();
		drivetrain.stop();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		control.update();
		
		// Tankdrive		
		drivetrain.joystickControl(left, right); //TODO calibrate joysticks
		
		// set in/outtake to position outdown = a, outup = b inup = x
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.A)) {
			take.setPosition(Take.Position.OUT_DOWN);
		} else if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.B)) {
			take.setPosition(Take.Position.OUT_UP);
		} else if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.X)) {
			take.setPosition(Take.Position.IN_UP);
		}

		// intake motor mapped to LB and RB
		if(operator.getRawButton(ControllerBase.GamepadButtons.LB))
		{
			take.setSpin(1);
		}
		else if(operator.getRawButton(ControllerBase.GamepadButtons.RB))
		{
			take.setSpin(-1);
		}
		else
		{
			take.setSpin(0);
		}
		
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
		if(control.getPressedDown(ControllerBase.Joysticks.GAMEPAD,ControllerBase.GamepadButtons.START))
		{
			if(basinFlag)
			{
				take.setBasinPosition(Take.basinPosition.Up);
				basinFlag = false;
			}
			else
			{
				take.setBasinPosition(Take.basinPosition.Down);
				basinFlag = true;
			}
		}

		// Climber bound to y but can only be actived 2 minutes into the match
		if (Timer.getMatchTime() >= 120) {
			double set = .5;
			if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.Y)) {
				take.setClimb(set);
				set += .005;
			}
			else 
			{
				take.setClimb(0);
			}
		}
		// Camera *Sigh*

		updateToSmartDash();
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

	@Override
	public void disabledPeriodic() {	
		control.update();
		
		if (control.getPressedDown(ControllerBase.Joysticks.GAMEPAD, ControllerBase.GamepadButtons.R3)) {
			gyro.calibrate();
			gyro.reset();
		}
		
		updateToSmartDash();
		//SmartDashboard.putBoolean("isCompromised?", DriverStation.getInstance().isDisabled());
	}
	
	public void updateToSmartDash()
	{
		// Send Gyro val to Dashboard
        SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
        // send the gear status to dashboard
        SmartDashboard.putBoolean("Gear Good?", camera.checkForGearLift());
        SmartDashboard.putNumber("Right Enc Value", drivetrain.getREncVal());
        SmartDashboard.putNumber("Left Enc Value", drivetrain.getLEncVal());
        SmartDashboard.putBoolean("isCompromised?", DriverStation.getInstance().isDisabled());
        SmartDashboard.putNumber("Distance to Target", camera.getDistanceToCenterOfTargets());
        SmartDashboard.putNumber("Angle to Target", camera.getAngleToTurnToCenterOfTargets());
	}
}
