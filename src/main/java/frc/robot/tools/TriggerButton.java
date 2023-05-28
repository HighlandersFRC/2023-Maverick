// Copyrights (c) 2018-2019 FIRST, 2020 Highlanders FRC. All Rights Reserved.

package frc.robot.tools;

import edu.wpi.first.wpilibj.GenericHID;
import java.util.function.BooleanSupplier;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class TriggerButton extends Trigger {
    public TriggerButton(BooleanSupplier bs){
        super(bs);
    }
}