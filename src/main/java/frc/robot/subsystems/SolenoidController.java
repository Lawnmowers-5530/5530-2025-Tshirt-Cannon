// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.Constants.PneumaticConstants;

public class SolenoidController extends SubsystemBase {
	PneumaticHub pHub;
	/**
	 * Controls flow from the high pressure tank to the low pressure tank
	 */
	private Solenoid hp;
	/**
	 * Used to open the quick dump valve
	 */
	private Solenoid qd;

	/**
	 * Open flow to shirt 1 valve
	 */
	private Solenoid shirt1;
	/**
	 * Open flow to shirt 2 valve
	 */
	private Solenoid shirt2;
	/**
	 * Open flow to shirt 3 valve
	 */
	private Solenoid shirt3;
	/**
	 * Open flow to shirt 4 valve
	 */
	private Solenoid shirt4;
	/**
	 * Open flow to shirt 5 valve
	 */
	private Solenoid shirt5;
	/**
	 * Open flow to shirt 6 valve
	 */
	private Solenoid shirt6;

	/**
	 * Keep track of the last shirt # fired
	 */
	private int lastShirt;

	/**
	 * Class for controlling the 8 solenoid setup for shooting
	 */
	public SolenoidController() {
		pHub = new PneumaticHub(PneumaticConstants.hub_id);

		hp = pHub.makeSolenoid(PneumaticConstants.hp_channel);

		qd = pHub.makeSolenoid(PneumaticConstants.qd_channel);
		qd.setPulseDuration(0.5);

		shirt1 = pHub.makeSolenoid(PneumaticConstants.shirt1_channel);
		shirt2 = pHub.makeSolenoid(PneumaticConstants.shirt2_channel);
		shirt3 = pHub.makeSolenoid(PneumaticConstants.shirt3_channel);
		shirt4 = pHub.makeSolenoid(PneumaticConstants.shirt4_channel);
		shirt5 = pHub.makeSolenoid(PneumaticConstants.shirt5_channel);
		shirt6 = pHub.makeSolenoid(PneumaticConstants.shirt6_channel);

		lastShirt = 0;
	}

	public InstantCommand closeAllSolenoids = new InstantCommand(
			() -> {
				hp.set(false);
				qd.set(false);
				shirt1.set(false);
				shirt2.set(false);
				shirt3.set(false);
				shirt4.set(false);
				shirt5.set(false);
				shirt6.set(false);
			});
	/**
	 * Generates a sequential command group to fill
	 */
	public SequentialCommandGroup fill = closeAllSolenoids
			.andThen(
					new InstantCommand(
							() -> {

								// opens the high pressure valve only if all other valves are closed to prevent
								// leaking
								if (!qd.get() && !shirt1.get() && !shirt2.get() && !shirt3.get() && !shirt4.get()
										&& !shirt5.get()
										&& !shirt6.get()) {
									hp.set(true);
									System.out.println("successful fill command");
								}
							},
							this))
			.andThen(
					// gives the high pressure tank 0.5 seconds to load the low pressure tank
					new WaitCommand(0.5))
			.andThen(
					new InstantCommand(() -> {
						hp.set(false);
					}));

	/**
	 * Generate a command to shoot
	 * 
	 * @param shirt The shirt number to fire
	 * @return A new {@link SequentialCommandGroup} to fill the tank and then shoot
	 *         the given shirt number
	 */
	public SequentialCommandGroup getShootCommand() {
		return fill.andThen(
				new WaitCommand(1))
				.andThen(
						new InstantCommand(
								() -> {
									switch (lastShirt + 1) {
										case 1:
											shirt1.set(true);
										case 2:
											shirt2.set(true);
										case 3:
											shirt3.set(true);
										case 4:
											shirt4.set(true);
										case 5:
											shirt5.set(true);
										case 6:
											shirt6.set(true);
									}
									qd.startPulse();
									this.lastShirt = (this.lastShirt + 1) % 6;
								},
								this));
	}

	/**
	 * Generate a command to shoot
	 * 
	 * @param shirt The shirt number to fire
	 * @return A new {@link SequentialCommandGroup} to fill the tank and then shoot
	 *         the given shirt number
	 */
	public SequentialCommandGroup getShootCommand(int shirt) throws IndexOutOfBoundsException {
		if (shirt < 1 || shirt > 6) {
			throw new IndexOutOfBoundsException();
		}
		return fill.andThen(
				new WaitCommand(1))
				.andThen(
						new InstantCommand(
								() -> {
									// set the last shirt fired to the given shirt
									this.lastShirt = shirt;

									switch (shirt) {
										case 1:
											shirt1.set(true);
										case 2:
											shirt2.set(true);
										case 3:
											shirt3.set(true);
										case 4:
											shirt4.set(true);
										case 5:
											shirt5.set(true);
										case 6:
											shirt6.set(true);
									}
									qd.startPulse();
								},
								this));
	}

	@Override public void periodic() {
		// This method will be called once per scheduler run
	}
}
