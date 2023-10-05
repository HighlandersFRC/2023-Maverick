// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Add your docs here. */
public class TriggerButton extends Trigger{
    public TriggerButton(BooleanSupplier bs) {
		super(bs);
	}
}
