// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.SwerveModule;

/** Add your docs here. */
public final class Constants {
	public static final class SwerveConstants {
		public static final double trackWidth = Units.inchesToMeters(24);
		public static final double wheelBase = Units.inchesToMeters(24);

		public static final class Mod0 { // FR
			public static final int driveMotor = 0;
			public static final int turnMotor = 0;
			public static final int canCoder = 0;
			public static final double angleOffset = 0;
		}

		public static final class Mod1 { // RL
			public static final int driveMotor = 0;
			public static final int turnMotor = 0;
			public static final int canCoder = 0;
			public static final double angleOffset = 0;
		}

		public static final class Modules {
			public static final SwerveModule Mod_0 = new SwerveModule(Mod0.driveMotor, Mod0.turnMotor, Mod0.canCoder,
					Mod0.angleOffset); // FL
			public static final SwerveModule Mod_1 = new SwerveModule(Mod1.driveMotor, Mod1.turnMotor, Mod1.canCoder,
					Mod1.angleOffset); // RL

		}

		public static final class SwerveModuleConstants {
			public static final double conversionFactor = -1 * (1 / 6.75) * Units.inchesToMeters(Math.PI * 4);

			public static final class SwerveAnglePIDConstants {
				public static final double p = 0.0075;
				public static final double i = 0.0025;
				public static final double d = 0;
			}
		}

		public static final class RotationConstants {
			public static final double kP = 0.75;
			public static final double kI = 0.0;
			public static final double kD = 0.0;
		}

		public static Translation2d m0 = new Translation2d(Constants.SwerveConstants.trackWidth / 2,
				Constants.SwerveConstants.wheelBase / 2);
		public static Translation2d m1 = new Translation2d(Constants.SwerveConstants.trackWidth / 2,
				-Constants.SwerveConstants.wheelBase / 2);
		public static Translation2d m2 = new Translation2d(-Constants.SwerveConstants.trackWidth / 2,
				Constants.SwerveConstants.wheelBase / 2);
		public static Translation2d m3 = new Translation2d(-Constants.SwerveConstants.trackWidth / 2,
				-Constants.SwerveConstants.wheelBase / 2);
		public static SwerveDriveKinematics kinematics = new SwerveDriveKinematics(m0, m1, m2, m3);
	}

	public static final class PneumaticConstants {
		public static final int hub_id = 0; // pneumatic hub id
		public static final int hp_channel = 0; // high pressure solenoid channel
		public static final int qd_channel = 0; // dump solenoid channel
		
		public static final int shirt1_channel = 0;
		public static final int shirt2_channel = 0;
		public static final int shirt3_channel = 0;
		public static final int shirt4_channel = 0;
		public static final int shirt5_channel = 0;
		public static final int shirt6_channel = 0;
	}

	public static final class ControllerConstants {
		public static final double driveControllerJoystickDeadband = 0.06;
	}
}
