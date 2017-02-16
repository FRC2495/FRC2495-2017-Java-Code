package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;

public class Take {

	DoubleSolenoid outin, downup;

	public enum Position {
		IN_UP, OUT_DOWN, OUT_UP;
	}

	public Take() {
		outin = new DoubleSolenoid(6, 1, 0);
		downup = new DoubleSolenoid(6, 2, 3);
	}

	public void setPosition(Position pos) {
		switch (pos) {
			case IN_UP: {
				downup.set(DoubleSolenoid.Value.kReverse);
				outin.set(DoubleSolenoid.Value.kReverse);
			}
			case OUT_DOWN: {
				outin.set(DoubleSolenoid.Value.kForward);
				downup.set(DoubleSolenoid.Value.kForward);
			}
			case OUT_UP: {
				outin.set(DoubleSolenoid.Value.kForward);
				downup.set(DoubleSolenoid.Value.kReverse);
			}
		}
	}

}
