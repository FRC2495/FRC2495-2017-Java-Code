package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.Timer;

public class HMCamera extends Robot {

	NetworkTable nt;
	double[][] returned = new double[5][];

	public HMCamera(String networktable) {
		nt = NetworkTable.getTable(networktable);
	}

	private void updateFromNT() {
		double[] def = {}; // Return an empty array by default.
		// Get data from NetworkTable
		returned[0] = nt.getNumberArray("area", def);
		returned[1] = nt.getNumberArray("width", def);
		returned[2] = nt.getNumberArray("height", def);
		returned[3] = nt.getNumberArray("centerX", def);
		returned[4] = nt.getNumberArray("centerY", def);
	}

	public boolean checkForGear()
	{
		updateFromNT();
		return(returned[0] != null);
	}
}
