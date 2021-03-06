package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.Timer;
//import edu.wpi.first.wpilibj.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class HMCamera {
	private static final int BAD_INDEX = -1;
	
	NetworkTable nt;
	double[] area, width, height, centerX, centerY;
	int largeAIndex, largeBIndex = BAD_INDEX;

	private static final int HORIZONTAL_CAMERA_RES_PIXELS = 320;
	private static final int VERTICAL_CAMERA_RES_PIXELS = 240;
	private static final double VERTICAL_FOV_DEGREES = 47;
	private static final double HORIZONTAL_FOV_DEGREES = 56;
	private static final int TARGET_HEIGHT_INCHES = 5;
	private static final double TARGET_WIDTH_INCHES = 2;

	private static final int MAX_NT_RETRY = 5;
	private static final double CAMERA_CATCHUP_DELAY_SECS = 0.250;

	public HMCamera(String networktable) {
		// nt = NetworkTable.getTable(networktable);
		nt = NetworkTableInstance.getDefault().getTable(networktable);
	}

	private void setLocalTables(double[] area, double[] width, double[] height, double[] centerX, double[] centerY) {
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
		largeAIndex = BAD_INDEX;
		largeBIndex = BAD_INDEX;

		// We cannot get arrays atomically but at least we can make sure they
		// have the same size
		do {
			// Get data from NetworkTable
			//setLocalTables(nt.getNumberArray("area", def), nt.getNumberArray("width", def),
			//		nt.getNumberArray("height", def), nt.getNumberArray("centerX", def),
			//		nt.getNumberArray("centerY", def));			
			setLocalTables(nt.getEntry("area").getDoubleArray(def), nt.getEntry("width").getDoubleArray(def),
					nt.getEntry("height").getDoubleArray(def), nt.getEntry("centerX").getDoubleArray(def),
					nt.getEntry("centerY").getDoubleArray(def));

			retry_count++;
		} while (!isCoherent() && retry_count < MAX_NT_RETRY);
	}

	private void processInformation() {
		double[] areaSave = area;
		if (areaSave.length >= 2) {
			largeAIndex = 0;
			largeBIndex = 0;
	        
	        //Checking first two elements of input array
	        if(areaSave[0] > areaSave[1])
	        {
	            //If first element is greater than second element
	            largeAIndex = 0;
	            largeBIndex = 1;
	        }
	        else
	        {
	            //If second element is greater than first element
	            largeAIndex = 1;
	            largeBIndex = 0;
	        }
	 
	        //Checking remaining elements of input array
	        for (int i = 2; i < areaSave.length; i++)
	        {
	            if(areaSave[i] > areaSave[largeAIndex])
	            {
	                //If element at 'i' is greater than 'firstLargest'
	                largeBIndex = largeAIndex;
	                largeAIndex = i;
	            }
	            else if (/*areaSave[i] < areaSave[largeAIndex] &&*/ areaSave[i] > areaSave[largeBIndex])
	            {
	                //If element at 'i' is smaller than 'firstLargest' and greater than 'secondLargest'
	                largeBIndex = i;
	            }
	        }
		}
	}

	public boolean isCoherent() {
		boolean result = (area != null && width != null && height != null && centerX != null && centerY != null
				&& area.length == width.length && area.length == height.length && area.length == centerX.length
				&& area.length == centerY.length);
		return result;
	}

	public int getNumberOfTargets() {
		if (isCoherent()) {
			int number = area.length;
			return number; // all tables have the same size so any length
								// can be used (might be zero)
		} else {
			//System.out.println("cannot get number of targets");
			return 0; // best answer in that case
		}
	}

	public boolean acquireTargets(boolean waitForNewInfo) {
		if (waitForNewInfo) {
			Timer.delay(CAMERA_CATCHUP_DELAY_SECS);
		}
		
		updateFromNT(); // gets the latest info

		if (isCoherent() && getNumberOfTargets() > 0) { // if we have targets
			processInformation();
			return true;
		} else {
			return false;
		}
	}

	public boolean checkForGearLift() {
		return getNumberOfTargets() > 1; // gear lift is at least two targets
	}

	public double getDistanceToTargetA() {
		if (isCoherent() && largeAIndex != BAD_INDEX) {
			double diagTargetDistance = TARGET_HEIGHT_INCHES * (VERTICAL_CAMERA_RES_PIXELS / height[largeAIndex]) / 2.0
					/ Math.tan(Math.toRadians(VERTICAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}
	
	public double getDistanceToTargetAUsingHorizontalFov()
	{
		if (isCoherent() && largeAIndex != BAD_INDEX) {
			double diagTargetDistance = TARGET_WIDTH_INCHES * (HORIZONTAL_CAMERA_RES_PIXELS / width[largeAIndex]) / 2.0
					/ Math.tan(Math.toRadians(HORIZONTAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}

	public double getDistanceToTargetB() {
		if (isCoherent() && largeBIndex != BAD_INDEX) {
			double diagTargetDistance = TARGET_HEIGHT_INCHES * (VERTICAL_CAMERA_RES_PIXELS / height[largeBIndex]) / 2.0
					/ Math.tan(Math.toRadians(VERTICAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}
	
	public double getDistanceToTargetBUsingHorizontalFov()
	{
		if (isCoherent() && largeBIndex != BAD_INDEX) {
			double diagTargetDistance = TARGET_WIDTH_INCHES * (HORIZONTAL_CAMERA_RES_PIXELS / width[largeBIndex]) / 2.0
					/ Math.tan(Math.toRadians(HORIZONTAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}

	public double getAngleToTurnToA() {
		if (isCoherent() && largeAIndex != BAD_INDEX) {
			double diff = (getCenterX()[largeAIndex] - (HORIZONTAL_CAMERA_RES_PIXELS / 2))
					/ HORIZONTAL_CAMERA_RES_PIXELS;
			double angle = diff * HORIZONTAL_FOV_DEGREES;
			return angle;
		} else
			return 0;
	}

	public double getAngleToTurnToB() {
		if (isCoherent() && largeBIndex != BAD_INDEX) {
			double diff = (getCenterX()[largeBIndex] - (HORIZONTAL_CAMERA_RES_PIXELS / 2))
					/ HORIZONTAL_CAMERA_RES_PIXELS;
			double angle = diff * HORIZONTAL_FOV_DEGREES;
			return angle;
		} else
			return 0;
	}
	
	public double getDistanceToCenterOfTargets()
	{
		return (getDistanceToTargetA() + getDistanceToTargetB()) / 2;
	}
	
	public double getDistanceToCenterOfTargetsUsingHorizontalFov()
	{
		return ((getDistanceToTargetAUsingHorizontalFov() + getDistanceToTargetBUsingHorizontalFov()) /2);
	}
	
	public double getAngleToTurnToCenterOfTargets()
	{
		return (getAngleToTurnToA() + getAngleToTurnToB()) / 2;
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
