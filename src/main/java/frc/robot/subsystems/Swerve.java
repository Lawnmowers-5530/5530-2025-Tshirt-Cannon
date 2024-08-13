// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import com.revrobotics.CANSparkBase.IdleMode;

import edu.wpi.first.math.Vector;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;
import frc.robot.data.GlobalState;

/**
 * Drivetrain control subsystem. Uses {@link SwerveModule}s to control the
 * movement of the robot. Also handles autonomous routines.
 */
public class Swerve extends SubsystemBase {

	private PIDController rotationPID;

	private static SwerveModule Mod_0;
	private static SwerveModule Mod_1;
	private static SwerveModule Mod_2;
	private static SwerveModule Mod_3;

	private boolean isCoasting;

	/**
	 * Initialize all swerve elements
	 */
	public Swerve() {
		Mod_0 = SwerveConstants.Modules.Mod_0;
		Mod_1 = new SwerveModule(0, 0, 0, 0); // filler swerve modules bc only FL and BR are driven
		Mod_2 = new SwerveModule(0, 0, 0, 0);
		Mod_3 = SwerveConstants.Modules.Mod_1;

		rotationPID = new PIDController(
				SwerveConstants.RotationConstants.kP,
				SwerveConstants.RotationConstants.kI,
				SwerveConstants.RotationConstants.kD);
	}

	/**
	 * Repeat call drive with supplier outputs
	 * 
	 * @param vector
	 * @param omegaRadSecSupplier
	 * @param fieldRelative
	 * @return {@link RunCommand} to drive with suppliers
	 */
	public Command supplierDrive(Supplier<Vector<N2>> vector, DoubleSupplier omegaRadSecSupplier,
			boolean fieldRelative) {
		return new RunCommand(
				() -> {
					this.drive(vector.get(), omegaRadSecSupplier.getAsDouble(), fieldRelative);
				}, this);
	};

	/**
	 * 
	 * @param vector        {@link Vector2D} to drive to. Uses the FRC coordinate
	 *                      system:
	 *                      i-hat [0,1],
	 *                      j-hat [-1,0]
	 * @param omegaRadSec   CCW+ radians per second
	 * @param fieldRelative Determines if vector is driven relative to 0 deg gyro
	 *                      angle
	 */
	public void drive(Vector<N2> vector, double omegaRadSec, boolean fieldRelative) {
		ChassisSpeeds speeds = fieldRelative ? ChassisSpeeds.fromRobotRelativeSpeeds(
				new ChassisSpeeds(
						vector.get(0),
						vector.get(1),
						omegaRadSec),
				Pgyro.getRot())
				: new ChassisSpeeds(
						vector.get(0),
						vector.get(1),
						omegaRadSec);

		SwerveModuleState[] states = SwerveConstants.kinematics.toSwerveModuleStates(speeds);
		Mod_0.setState(states[0]);
		Mod_1.setState(states[1]);
		Mod_2.setState(states[2]);
		Mod_3.setState(states[3]);

	}

	/**
	 * Uses {@link PIDController} to reach rotation target
	 * 
	 * @return A RunCommand to rotate to the angle
	 */
	public Command rotateToAngle(Supplier<Vector<N2>> vectorSupplier, DoubleSupplier angleSupplier) {
		return new RunCommand(
				() -> {
					double rotationOutput = rotationPID.calculate(Pgyro.getRot().getRadians(),
							angleSupplier.getAsDouble());
					this.drive(vectorSupplier.get(), rotationOutput, false);
				},
				this).until(() -> {
					return rotationPID.atSetpoint();
				}); // lambda boolean supplier to detect if at rotation setpoint
	}

	/**
	 * Send {@link SwerveModuleState}s to SwerveModule objects
	 */
	public void setModuleStates(SwerveModuleState[] desiredStates) {
		Mod_0.setState(desiredStates[0]);
		Mod_1.setState(desiredStates[1]);
		Mod_2.setState(desiredStates[2]);
		Mod_3.setState(desiredStates[3]);
	}

	/**
	 * Updates odometry and disables coasting if robot enables
	 */
	@Override
	public void periodic() {
		if (isCoasting && GlobalState.isEnabled) {
			Mod_0.setIdleMode(IdleMode.kBrake);
			Mod_1.setIdleMode(IdleMode.kBrake);
			Mod_2.setIdleMode(IdleMode.kBrake);
			Mod_3.setIdleMode(IdleMode.kBrake);
			isCoasting = false;
		}
	}

	/**
	 * Controls if motors are in coast or brake mode depending on if robot is
	 * enabled
	 */
	public void disabledPeriodic() {
		if (!isCoasting) {
			isCoasting = true;
			Mod_0.setIdleMode(IdleMode.kCoast);
			Mod_1.setIdleMode(IdleMode.kCoast);
			Mod_2.setIdleMode(IdleMode.kCoast);
			Mod_3.setIdleMode(IdleMode.kCoast);
		}
	}

}
