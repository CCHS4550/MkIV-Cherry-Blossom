// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.helpers.OI;

public class Lights extends SubsystemBase {


  public enum LEDState {
    rainbow
  }

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(98);
  int rainbowFirstPixelHue = 1;

  
  /** Creates a new Lights. */
  public Lights() {
    leds.setLength(ledBuffer.getLength());
    leds.start();
  }

  public void lightsDefaultMethod() {
    this.changeLEDState(LEDState.rainbow);
    
  }

  private void changeLEDState(LEDState state) {


    switch(state) {
      case rainbow:

      setDefaultCommand(new RunCommand(() -> {
      // For every pixel
      for (var i = 0; i < ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
      final var hue = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
      // Set the value
      ledBuffer.setHSV(i, hue, 255, 128);
      }
      // Increase by to make the rainbow "move"
      rainbowFirstPixelHue += 1;
      // System.out.println(rainbowFirstPixelHue);
      // Check bounds
      rainbowFirstPixelHue %= 180;
      }, this));

      leds.setData(ledBuffer);

      
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
