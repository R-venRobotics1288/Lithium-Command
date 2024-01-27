package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;

/**
 * Subsystem for managing digital limit switches.
 */
public class LimitSwitchSubsystem extends SubsystemBase {
    private final DigitalInput[] limitSwitches;

    /**
     * Initialises Limit Switch subsystem specified in ModuleConstants
     */
    public LimitSwitchSubsystem() {
        limitSwitches = new DigitalInput[Constants.ModuleConstants.LIMIT_SWITCHES.length];
        for (int i = 0; i < limitSwitches.length; i++) {
            limitSwitches[i] = new DigitalInput(Constants.ModuleConstants.LIMIT_SWITCHES[i]);
        }
    }

    /**
     * Gets the status of a given limit switch.
     * @param index index of the switch to get, based on array in ModuleConstants
     * @return Boolean status of the given limit switch
     */
    public boolean isClosed(int index) {
        return limitSwitches[index].get();
    }
}