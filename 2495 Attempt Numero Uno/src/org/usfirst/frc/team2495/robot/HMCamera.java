package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.wpilibj.vision.*;
import edu.wpi.first.wpilibj.Timer;

public class HMCamera extends Robot {

	NetworkTable nt;
	double[] area, width, height, centerX, centerY;
	double[] def = {}; // Return an empty array by default.
	double largestRectNum;
	double largestRectArea;

	public HMCamera(String networktable) {
		nt = NetworkTable.getTable(networktable);
	}

	private void updateFromNT() {
		// Get data from NetworkTable
		area = nt.getNumberArray("area", def);
		width = nt.getNumberArray("width", def);
		height = nt.getNumberArray("height", def);
		centerX = nt.getNumberArray("centerX", def);
		centerY = nt.getNumberArray("centerY", def);
		
//		largestRectArea = area[0];
//		largestRectNum = 0;
//		for (int i = 1; i < area.length; i++) { // saves an iteration by
//													// starting at 1
//			if (area[i] >= largestRectArea) {
//				largestRectNum = i;
//			}
//		}
	}

	public boolean checkForGear()
	{
		updateFromNT();
		return (!area.equals(def));
	}
	
	public double[] getArea()
	{
		updateFromNT();
		return area;
	}
	
	public double[] getWidth()
	{
		updateFromNT();
		return width;
	}
	
	public double[] getHeight()
	{
		updateFromNT();
		return height;
	}
	
	public double[] getCenterX()
	{
		updateFromNT();
		return centerX;
	}
	
	public double[] getCenterY()
	{
		updateFromNT();
		return centerY;
	}
}
