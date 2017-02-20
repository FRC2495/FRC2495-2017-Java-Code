package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.CANTalon;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;

import java.util.ResourceBundle.Control;

import org.opencv.imgproc.Imgproc;
import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	final String BaseBreak = "Baseline Breaker"; // Auton selection cases
	final String GearGrab = "Gear Grabber";
	final String GearGrab2 = "Gear Grabber Right";
	final String FuelFling = "Fuel Flinger";
	final String DankDump = "Dank Dumper";
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

	// DoubleSolenoid extendpickup = new DoubleSolenoid(0, 1); // Solenoids
	// DoubleSolenoid droppickup = new DoubleSolenoid(2, 3);
	// DoubleSolenoid retracthood = new DoubleSolenoid(4, 5);

	// Relay compresscontrol = new Relay(0); // Spike that controls compressor
	ADXRS450_Gyro gyro; // gyro
	PowerDistributionPanel PDP = new PowerDistributionPanel(6); // PDP

	DriveTrain drivetrain; // DriveTrain object from the homemade class

	Timer time = new Timer(); // generic timer

	static final int IMG_WIDTH = 320; // height and width of the camera image
	static final int IMG_HEIGHT = 240;

	VisionThread visionthread; // sepearate vision thread
	double centerx = 0.0; // center of the x axis
	final Object imglock = new Object();

	Take take;

	ControllerBase control;

	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	@Override
	public void robotInit() {
		chooser = new SendableChooser();
		chooser.addDefault("Baseline Breaker", BaseBreak); // move forward
		chooser.addObject("Gear Grabber", GearGrab); // put the gear on the peg
		chooser.addObject("Gear Grabber Left Side", GearGrab2);
		chooser.addObject("Fuel Flinger", FuelFling); // shoot
		chooser.addObject("Dank Dumper", DankDump); // go to the hopper and then

		RR = new CANTalon(1); // The CANTalons
		RF = new CANTalon(2);
		LR = new CANTalon(3);
		LF = new CANTalon(4);
		// intake = new CANTalon(5);

		// dump

		drivetrain = new DriveTrain(RR, RF, LR, LF, gyro);
		camera = new HMCamera("myContoursReport");
		gyro = new ADXRS450_Gyro(SPI.Port.kOnboardCS0);

		Compressor compressor = new Compressor();
		compressor.checkCompressor();

		take = new Take();

		left = new Joystick(0);
		right = new Joystick(1);
		operator = new Joystick(2);

		control = new ControllerBase(operator, left, right);

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
		gyro.calibrate();
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

		
		  SmartDashboard.putNumber("Right Enc Value", drivetrain.getREncVal());
		  SmartDashboard.putNumber("Left Enc Value", drivetrain.getLEncVal());
		 

		switch (autoSelected) {
		case DankDump:
			// Put custom auto code here
			break;
		case FuelFling:
			// code
			break;
		case GearGrab: {
			if (camera.checkForGear()) {
				drivetrain.moveDistance(80);
			}
		}
			break;
		case GearGrab2: {
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
			// dropgear
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
				System.out.println("check if moving");
				drivetrain.moveDistance(12);
				 System.out.println("Check2");
				//drivetrain.waitMove();
				 //System.out.println("Check3");
				autoSelected = AutonDone;
			}
		}
			break;
		}
	}

	@Override
	public void teleopInit() {
		time.stop();
		time.reset();
	}

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {

		// Tankdrive
		drivetrain.joystickControl(left, right);

		control.update();

		// set in/outtake to position
		if (control.getPressedDown(ControllerBase.Joysticks.OPERATOR, ControllerBase.JoystickButtons.BTN1)) {
			take.setPosition(Take.Position.OUT_DOWN);
		} else if (control.getPressedDown(ControllerBase.Joysticks.OPERATOR, ControllerBase.JoystickButtons.BTN2)) {
			take.setPosition(Take.Position.OUT_UP);
		} else if (control.getPressedDown(ControllerBase.Joysticks.OPERATOR, ControllerBase.JoystickButtons.BTN3)) {
			take.setPosition(Take.Position.IN_UP);
		}

		// intake motor mapped to 4 and 5

		// Climber
		if (Timer.getMatchTime() >= 120) {
			if (control.getPressedDown(ControllerBase.Joysticks.OPERATOR, ControllerBase.JoystickButtons.BTN5)) {
				// trigger sequence of climbing
			}
		}
		// Camera *Sigh*

		// Send Gyro val to Dashboard
		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
		// send the gear status to dashboard
		SmartDashboard.putBoolean("Gear Good?", camera.checkForGear());
		SmartDashboard.putNumber("Right Enc Value", drivetrain.getREncVal());
		SmartDashboard.putNumber("Left Enc Value", drivetrain.getLEncVal());
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
		// Send Gyro val to Dashboard
		SmartDashboard.putNumber("Gyro Value", gyro.getAngle());
		// send the gear status to dashboard
		SmartDashboard.putBoolean("Gear Good?", camera.checkForGear());
		SmartDashboard.putNumber("Right Enc Value", drivetrain.getREncVal());
		SmartDashboard.putNumber("Left Enc Value", drivetrain.getLEncVal());
		if (operator.getTrigger()) {
			gyro.calibrate();
		}
	}
}
