package org.usfirst.frc.team2495.robot;

import java.util.Timer;
import java.util.TimerTask;

import edu.wpi.first.wpilibj.DoubleSolenoid;


public class Take {
	Timer timer;
	
	DoubleSolenoid outin, downup;

	public enum Position {
		IN_UP, OUT_DOWN, OUT_UP;
	}

	public Take() {
		outin = new DoubleSolenoid(6, 1, 0);
		downup = new DoubleSolenoid(6, 2, 3);
		timer = new Timer();
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

}
