package org.usfirst.frc.team2495.robot;

public class Ports {
	

		public static class Digital{
			public static final int CHECK_PRESSURE = 0;
			
		}
		
		public static class Relay{
			public static final int COMPRESSOR_RELAY = 0;
		}
		
		public static class CAN{
			public static final int RIGHT_REAR = 1;
			public static final int RIGHT_FRONT = 2;
			public static final int LEFT_REAR = 3;
			public static final int LEFT_FRONT = 4;
			public static final int SPIN = 5;
			public static final int CLIMB = 6;
			public static final int BASIN = 7;
			public static final int PCM = 8;
			public static final int PDP = 0;
		}
		
		public static class USB{
			public static final int RIGHT = 0;
			public static final int LEFT = 1;
			public static final int GAMEPAD = 2;
		}
		
		public static class PCM{
			public static final int INTAKE_IN = 0;
			public static final int INTAKE_OUT = 1;
			public static final int INTAKE_DOWN = 2;
			public static final int INTAKE_UP = 3;
			public static final int GEAR_IN = 4;
			public static final int GEAR_OUT = 5;
			public static final int BASIN_DOWN = 6;
			public static final int BASIN_UP = 7;
		
		}
}
