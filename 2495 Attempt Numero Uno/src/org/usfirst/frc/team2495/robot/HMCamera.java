package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class HMCamera{

	NetworkTable nt;
	double[] area, width, height, centerX, centerY;
	
	private static final int MAX_NT_RETRY = 5;

	public HMCamera(String networktable) {
		nt = NetworkTable.getTable(networktable);
	}

	private void setLocalTables(double[] area, double[] width, double[] height, double[] centerX,  double[] centerY) {
		this.area = area;
		this.width = width;
		this.height = height;
		this.centerX = centerX;
		this.centerY = centerY;
	}
	
	private void updateFromNT() {
		double[] def = {}; // Return an empty array by default.		
		int retry_count = 0;
		setLocalTables(null, null, null, null, null);	

		// We cannot get arrays atomically but at least we can make sure they
		// have the same size
		do {
			// Get data from NetworkTable
			setLocalTables(
					nt.getNumberArray("area", def),
					nt.getNumberArray("width", def),
					nt.getNumberArray("height", def),
					nt.getNumberArray("centerX", def),
					nt.getNumberArray("centerY", def));

			retry_count++;
		} while (!isCoherent() && retry_count < MAX_NT_RETRY);
	}
	
	public boolean isCoherent() {
		return (area != null && width != null && height != null && centerX != null
				&& centerY != null && area.length == width.length
				&& area.length == height.length && area.length == centerX.length
				&& area.length == centerY.length);
	}
	
	public int getNumberOfTargets()
	{
		if (isCoherent()) {
			return area.length; // all tables have the same size so any length can be used (might be zero)
		} else {
			return 0; // best answer in that case
		}
	}
	
	public boolean acquireTargets() {
		updateFromNT(); // gets the latest info

		if (isCoherent() && getNumberOfTargets() > 0) { // if we have targets 
			return true;
		} else {
			return false;
		}
	}
 
	public boolean checkForGearLift() {
		return getNumberOfTargets() > 1; // gear lift is at least two targets
	}

	public double[] getArea() {
		return area;
	}

	public double[] getWidth() {
		return width;
	}

	public double[] getHeight() {
		return height;
	}

	public double[] getCenterX() {
		return centerX;
	}

	public double[] getCenterY() {
		return centerY;
	}
}
