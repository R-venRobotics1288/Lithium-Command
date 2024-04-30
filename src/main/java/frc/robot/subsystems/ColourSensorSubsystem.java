package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/** Subsystem for managing the ColourSensor subsystem. */
public class ColourSensorSubsystem extends SubsystemBase {
  private final ColorSensorV3 colorSensor;
  private int proximity =
      0; // rough proximity based on IR, goes from 0 - 2047 where 2047 is closest
  private Color detectedColor =
      new Color(); // estimated colour from sensor - more accurate the closer the object is

  /** Initialises the ColourSensor subsystem using COLOUR_SENSOR_PORT Module Constant. */
  public ColourSensorSubsystem() {
    colorSensor = new ColorSensorV3(Constants.ModuleConstants.COLOUR_SENSOR_PORT);
  }

  /**
   * Gets the current best guess proximity of the closest object to the colour sensor.
   *
   * @return INT32 proximity
   */
  public int getProximity() {
    return proximity;
  }

  /**
   * Gets the current best guess colour of the object closest to the colour sensor.
   *
   * @return RGBA Colour struct detectedColour
   */
  public Color getDetectedColour() {
    return detectedColor;
  }

  @Override
  public void periodic() {
    detectedColor = colorSensor.getColor();
    proximity = colorSensor.getProximity();
  }
}
