// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import frc.helpers.OI;

public class Lights extends SubsystemBase {

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(98);
  int rainbowFirstPixelHue = 1;
  /** Creates a new Lights. */
  public Lights() {
    leds.setLength(ledBuffer.getLength());
    
    leds.start();
    
  }

  public void lightsDefaultMethod(CommandXboxController controller){
      this.rainbow();
      leds.setData(ledBuffer);
  }

    
  

    private void rainbow() {

    // For every pixel
    for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
      
    }


          // Increase by to make the rainbow "move"
    rainbowFirstPixelHue += 3;
    System.out.println(rainbowFirstPixelHue);
    // Check bounds
    rainbowFirstPixelHue %= 180;
    

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
