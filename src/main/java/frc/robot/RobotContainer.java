// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.ControllerConstants;
import frc.robot.subsystems.SolenoidController;
import frc.robot.subsystems.Swerve;

public class RobotContainer {
  private Swerve swerve;
  private CommandXboxController driverController;
  private SolenoidController solenoidController;

  private Supplier<Vector<N2>> driveVectorSupplier;
  private DoubleSupplier driveRotationSupplier;

  public RobotContainer() {
    swerve = new Swerve();
    solenoidController = new SolenoidController();
    driverController = new CommandXboxController(0);

    driveVectorSupplier = () -> {
      return VecBuilder.fill(
          MathUtil.applyDeadband(
              this.driverController.getLeftY(),
              ControllerConstants.driveControllerJoystickDeadband,
              1),
          MathUtil.applyDeadband(
              -this.driverController.getLeftX(),
              ControllerConstants.driveControllerJoystickDeadband,
              1));
    };

  driveRotationSupplier= () -> {
			return MathUtil.applyDeadband(
					-this.driverController.getRightX(),
					ControllerConstants.driveControllerJoystickDeadband,
					1);
  };

  configureBindings();
}

  private void configureBindings() {
    final RunCommand swerveCommand = new RunCommand(() -> {
      this.swerve.supplierDrive(driveVectorSupplier, driveRotationSupplier, true);
    },
        swerve);
    swerve.setDefaultCommand(swerveCommand);

    //shoots the next shirt, starting with shirt one and looping over after shirt 6
    this.driverController.a().onTrue(this.solenoidController.getShootCommand());


  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
