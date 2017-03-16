package org.usfirst.frc.team2495.robot;

import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.CANTalon;


public class Take {
	Timer timer;
	CANTalon spin, climb;
	DoubleSolenoid outin, downup, gear;

	public enum Position {
		IN_UP, OUT_DOWN, OUT_UP;
	}
	
	public enum gearPosition
	{
		In,Out;
	}

	public Take(CANTalon spin_in, CANTalon climb_in) {
		outin = new DoubleSolenoid(Ports.CAN.PCM, Ports.PCM.INTAKE_OUT, Ports.PCM.INTAKE_IN);
		downup = new DoubleSolenoid(Ports.CAN.PCM, Ports.PCM.INTAKE_DOWN, Ports.PCM.INTAKE_UP);
		gear = new DoubleSolenoid(Ports.CAN.PCM, Ports.PCM.GEAR_IN, Ports.PCM.GEAR_OUT);
		timer = new Timer();
		spin = spin_in;
		spin.enableBrakeMode(true);
		spin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
		climb = climb_in;
		climb.enableBrakeMode(true);
		climb.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
	}
	
	private TimerTask schedule(final Runnable r, long delay)
	{
		final TimerTask task = new TimerTask() 
			{
				public void run()
				{
					r.run();
				}
			};
			timer.schedule(task,delay);
			return task;
	}
	

	public void setPosition(Position pos) {
		switch (pos) {
			case IN_UP: {
				downup.set(DoubleSolenoid.Value.kReverse);
				schedule(() -> outin.set(DoubleSolenoid.Value.kForward), 500);
				break;
			}
			case OUT_DOWN: {
				outin.set(DoubleSolenoid.Value.kReverse);
				schedule(() -> downup.set(DoubleSolenoid.Value.kForward), 250);
				break;
			}
			case OUT_UP: {
				outin.set(DoubleSolenoid.Value.kReverse);
				downup.set(DoubleSolenoid.Value.kReverse);
				break;
			}
		}
	}
	
	public void setSpin(double speed)
	{
		spin.set(speed);
	}
	
	public void setClimb(double speed)
	{
		climb.set(speed);
	}
	
	public void setGearPosition(gearPosition pos)
	{
		switch(pos)
		{
			case In:
			{
				gear.set(DoubleSolenoid.Value.kReverse);
				break;
			}
			case Out:
			{
				gear.set(DoubleSolenoid.Value.kForward);
				break;
			}
		}
	}

}
