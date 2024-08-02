// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  record led(int point, int magnitude){}

  public enum LEDState {
    rainbow,
    pinkSolid,
    pinkWhiteGradient
  }

  AddressableLED leds = new AddressableLED(0);
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(175);
  int rainbowFirstPixelHue = 1;
  int pinkWhiteGradientFirstPixelHue = 1;

  /** Creates a new Lights. */
  public Lights() {
    leds.setLength(ledBuffer.getLength());
    leds.start();
  }

  public void lightsDefaultMethod() {
    this.changeLEDState(LEDState.rainbow);
  }

  public void changeLEDState(LEDState state) {

    switch (state) {
      case rainbow:
        RunCommand rainbowCommand = new RunCommand(() -> {
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
          }, this);

        rainbowCommand.schedule();
        leds.setData(ledBuffer);
        
        break;

      case pinkSolid:
        RunCommand pinkSolidCommand = new RunCommand(() -> {
          for (var i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setLED(i, Color.kLightPink);
          }
          }, this);

        pinkSolidCommand.schedule();
        leds.setData(ledBuffer);

        break;

      case pinkWhiteGradient:
        RunCommand pinkWhiteGradientCommand = new RunCommand(() -> {
          // For every pixel
          for (var i = 0; i < ledBuffer.getLength(); i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var saturation = (rainbowFirstPixelHue + (i * 255 / ledBuffer.getLength())) % 255;
          // Set the value
          ledBuffer.setHSV(i, 180, saturation, 128);
          }
          // Increase by to make the rainbow "move"
          pinkWhiteGradientFirstPixelHue += 1;
          // System.out.println(rainbowFirstPixelHue);
          // Check bounds
          pinkWhiteGradientFirstPixelHue %= 255;
          }, this);

        pinkWhiteGradientCommand.schedule();
        leds.setData(ledBuffer);

        break;
      
            
    }
  }

  private void pinkPulse() {

    int randomLED = (int)(Math.random() * ledBuffer.getLength());
    
    int r = 255;
    int g = 220;
    int b = 255;
    while (g > 0) {

    }

  }

  private void spread(int mainPoint) {


    for(int i = 0; i < 5; i++) {


    }
    



  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}
