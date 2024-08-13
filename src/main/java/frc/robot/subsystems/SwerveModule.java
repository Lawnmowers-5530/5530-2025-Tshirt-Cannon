// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants.SwerveConstants.SwerveModuleConstants.SwerveAnglePIDConstants;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
	private PIDController anglePID = new PIDController(SwerveAnglePIDConstants.p, SwerveAnglePIDConstants.i,
			SwerveAnglePIDConstants.d);
	private double pidOut;
	private CANSparkMax drive;
	private CANSparkMax rotate;
	private RelativeEncoder encoder;
	private CANcoder canCoder;
	private double angleOffset;

	public SwerveModule(int driveMotorID, int turnMotorID, int canCoderID, double angleOffset) { // initialize module

		drive = new CANSparkMax(driveMotorID, MotorType.kBrushless);
		rotate = new CANSparkMax(turnMotorID, MotorType.kBrushless);
		drive.setIdleMode(IdleMode.kBrake);
		rotate.setIdleMode(IdleMode.kBrake);

		this.canCoder = new CANcoder(canCoderID);
		encoder = drive.getEncoder();
		encoder.setPosition(0);
		;
		anglePID.enableContinuousInput(0, 360);
		this.angleOffset = angleOffset;

	}

	public void setState(SwerveModuleState state) {
		state = SwerveModuleState.optimize(state, getTurningPosition());

		drive.set(state.speedMetersPerSecond);
		pidOut = anglePID.calculate(getTurningPosition().getDegrees(), state.angle.getDegrees());
		rotate.set(pidOut);
	}

	public Rotation2d getTurningPosition() {
		return new Rotation2d(
				(this.canCoder.getAbsolutePosition().getValue() * 360 + this.angleOffset) * (Math.PI / 180));
	}

	public double getVelocity() { // rpm to rotations / second
		return encoder.getVelocity() * Constants.SwerveConstants.SwerveModuleConstants.conversionFactor / 60;
	}

	public double getOffset() {
		return this.angleOffset;
	}

	public double getDistance() {
		return encoder.getPosition() * Constants.SwerveConstants.SwerveModuleConstants.conversionFactor;
	}

	public SwerveModulePosition getPos() {
		return new SwerveModulePosition(getDistance(), getTurningPosition());
	}

	public SwerveModuleState getState() {
		return new SwerveModuleState(getVelocity(), getTurningPosition());
	}

	public void setIdleMode(IdleMode idlemode) {
		drive.setIdleMode(idlemode);
		rotate.setIdleMode(idlemode);
	}

}