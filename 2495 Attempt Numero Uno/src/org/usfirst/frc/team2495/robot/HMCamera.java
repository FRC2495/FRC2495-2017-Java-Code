package org.usfirst.frc.team2495.robot;

import edu.wpi.first.wpilibj.networktables.NetworkTable;

public class HMCamera {

	NetworkTable nt;
	double[] area, width, height, centerX, centerY;
	int largeAIndex, largeBIndex = 0;

	private static final int HORIZONTAL_CAMERA_RES_PIXELS = 320;
	private static final int VERTICAL_CAMERA_RES_PIXELS = 240;
	private static final double VERTICAL_FOV_DEGREES = 58;
	private static final double HORIZONTAL_FOV_DEGREES = 58;
	private static final int TARGET_HEIGHT_INCHES = 5;
	private static final double TARGET_WIDTH_INCHES = 10.5;

	private static final int MAX_NT_RETRY = 5;

	public HMCamera(String networktable) {
		nt = NetworkTable.getTable(networktable);
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
		largeAIndex = 0;
		largeBIndex = 0;

		// We cannot get arrays atomically but at least we can make sure they
		// have the same size
		do {
			// Get data from NetworkTable
			setLocalTables(nt.getNumberArray("area", def), nt.getNumberArray("width", def),
					nt.getNumberArray("height", def), nt.getNumberArray("centerX", def),
					nt.getNumberArray("centerY", def));

			retry_count++;
		} while (!isCoherent() && retry_count < MAX_NT_RETRY);
	}

	private void processInformation() {
		double[] areaSave = area;
		if (areaSave.length >= 2) {
			largeAIndex = 0;
			for (int i = 1; i < areaSave.length; i++) {
				if (areaSave[i] > areaSave[largeAIndex]) {
					largeAIndex = i;
				}
			}

			for (int i = 0; i < areaSave.length; i++) {
				if (areaSave[i] > areaSave[largeBIndex] && i != largeAIndex) {
					largeBIndex = i;
				}
			}
		}

		// step 1: verify we have at least two targets Done

		// step 2: get the index of the largest two targets Done

		// step 3: calculate the bounding box of the largest two targets (i.e.
		// the small rectangle that encompasses the largest two targets)

		// step 4: calculate the distance to the virtual target represented by
		// the bounding box of the largest two targets

		// step 5: calculate the angle delta between the current robot heading
		// and the center of the virtual target
	}

	public boolean isCoherent() {
		return (area != null && width != null && height != null && centerX != null && centerY != null
				&& area.length == width.length && area.length == height.length && area.length == centerX.length
				&& area.length == centerY.length);
	}

	public int getNumberOfTargets() {
		if (isCoherent()) {
			return area.length; // all tables have the same size so any length
								// can be used (might be zero)
		} else {
			return 0; // best answer in that case
		}
	}

	public boolean acquireTargets() {
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
		if (isCoherent() && largeAIndex != 0) {
			double diagTargetDistance = TARGET_HEIGHT_INCHES * (VERTICAL_CAMERA_RES_PIXELS / height[largeAIndex]) / 2.0
					/ Math.tan(Math.toRadians(VERTICAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}

	public double getDistanceToTargetB() {
		if (isCoherent() && largeBIndex != 0) {
			double diagTargetDistance = TARGET_HEIGHT_INCHES * (VERTICAL_CAMERA_RES_PIXELS / width[largeBIndex]) / 2.0
					/ Math.tan(Math.toRadians(VERTICAL_FOV_DEGREES / 2));
			return diagTargetDistance;
		} else
			return Double.POSITIVE_INFINITY;
	}

	public double getAngleToTurnToA() {
		if (isCoherent() && largeAIndex != 0) {
			double diff = (getCenterX()[largeAIndex] - (HORIZONTAL_CAMERA_RES_PIXELS / 2))
					/ HORIZONTAL_CAMERA_RES_PIXELS;
			double angle = diff * HORIZONTAL_FOV_DEGREES;
			return angle;
		} else
			return Double.POSITIVE_INFINITY;
	}

	public double getAngleToTurnToB() {
		if (isCoherent() && largeBIndex != 0) {
			double diff = (getCenterX()[largeBIndex] - (HORIZONTAL_CAMERA_RES_PIXELS / 2))
					/ HORIZONTAL_CAMERA_RES_PIXELS;
			double angle = diff * HORIZONTAL_FOV_DEGREES;
			return angle;
		} else
			return Double.POSITIVE_INFINITY;
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
