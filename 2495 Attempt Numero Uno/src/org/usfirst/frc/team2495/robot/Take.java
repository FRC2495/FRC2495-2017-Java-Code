package org.usfirst.frc.team2495.robot;

import java.util.Timer;
import java.util.TimerTask;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.CANTalon;


public class Take {
	Timer timer;
	CANTalon spin;
	DoubleSolenoid outin, downup, gear, basin;

	public enum Position {
		IN_UP, OUT_DOWN, OUT_UP;
	}
	
	public enum basinPosition
	{
		Up,Down;
	}
	
	public enum gearPosition
	{
		In,Out;
	}

	public Take(CANTalon Spin) {
		outin = new DoubleSolenoid(6, 1, 0);
		downup = new DoubleSolenoid(6, 2, 3);
		gear = new DoubleSolenoid(6, 4, 5);
		basin = new DoubleSolenoid(6, 6, 7);
		timer = new Timer();
		spin = Spin;
		spin.enableBrakeMode(true);
		spin.changeControlMode(CANTalon.TalonControlMode.PercentVbus);
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
	
	public void setBasinPosition(basinPosition pos)
	{
		switch(pos)
		{
			case Up:
				{
					basin.set(DoubleSolenoid.Value.kForward);
					break;
				}
			case Down:
				{
					basin.set(DoubleSolenoid.Value.kReverse);
					break;
				}
		}
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
