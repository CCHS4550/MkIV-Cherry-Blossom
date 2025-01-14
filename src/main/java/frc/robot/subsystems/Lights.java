// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Lights extends SubsystemBase {

  public static Lights mInstance;

  public static Lights getInstance() {
    if (mInstance == null) {
      mInstance = new Lights();
    }
    return mInstance;
  }

  PneumaticsSystem pneumatics;

  // "slope" is the ratio of the PSI to the Hue.
  // ps. Couldn't be bothered to understand the math, but I used this
  // https://stackoverflow.com/questions/5731863/mapping-a-numeric-range-onto-another
  private float slope = (float) ((80.0 - 10) / (70.0 - 0));
  private float inputPSI;
  private int outputHue;

  boolean brightening = true;

  public enum LEDState {
    rainbow,
    pinkSolid,
    pinkWhiteGradient,
    pressureRedtoGreenGradient
  }

  AddressableLED leds = new AddressableLED(0);
  /* Total LED length = 169 LEDS
   * LED 93-169 - The Ring around the barrels
   */
  AddressableLEDBuffer ledBuffer = new AddressableLEDBuffer(169);
  int rainbowFirstPixelHue = 1;
  int pinkWhiteGradientFirstPixelSaturation = 1;
  int pressureRedtoGreenGradientFirstPixelSaturation = 1;

  /** Creates a new Lights. */
  private Lights() {

    leds.setLength(ledBuffer.getLength());
    leds.start();
  }

  public void getLEDStateCommand(LEDState state) {

    switch (state) {
      default:
        // return new InstantCommand(() -> System.out.println("None!")).withName("Null");
      case rainbow:
        // WrapperCommand rainbowCommand =
        //     new RunCommand(
        //             () -> {
        // For every pixel
        for (var i = 0; i < 93; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var hue1 = (rainbowFirstPixelHue + (i * 180 / ledBuffer.getLength())) % 180;
          // Set the value
          ledBuffer.setHSV(i, hue1, 255, 128);
        }
        // Increase by to make the rainbow "move"
        rainbowFirstPixelHue += 1;
        // System.out.println(rainbowFirstPixelHue);
        // Check bounds
        rainbowFirstPixelHue %= 180;
        leds.setData(ledBuffer);
        //             },
        //             this)
        //         .withName("Rainbow Lights");

        // return rainbowCommand;
        // rainbowCommand.schedule();
        // System.out.println("rainbow!");

      case pinkSolid:
        // WrapperCommand pinkSolidCommand =
        //     new RunCommand(
        //             () -> {
        for (var i = 0; i < ledBuffer.getLength(); i++) {
          ledBuffer.setLED(i, Color.kLightPink);
        }
        //             },
        //             this)
        //         .withName("Solid Pink Lights");

        // return pinkSolidCommand;

        // pinkSolidCommand.schedule();
        // leds.setData(ledBuffer);

      case pinkWhiteGradient:
        // WrapperCommand pinkWhiteGradientCommand =
        //     new RunCommand(
        //             () -> {
        var saturation1 = 0;
        // For every pixel
        for (var i = 0; i < 93; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          final var saturation =
              (pinkWhiteGradientFirstPixelSaturation + (i * 210 / ledBuffer.getLength())) % 210;
          // Set the value
          ledBuffer.setHSV(i, 180, 225, saturation);
        }
        // Increase by to make the rainbow "move"
        pinkWhiteGradientFirstPixelSaturation += 1;
        // System.out.println(rainbowFirstPixelHue);
        // Check bounds
        pinkWhiteGradientFirstPixelSaturation %= 210;

        //             },
        //             this)
        //         .withName("Pink White Gradient Lights");

        // return pinkWhiteGradientCommand;

        // pinkWhiteGradientCommand.schedule();
        // System.out.println("Pink White Gradient!");

      case pressureRedtoGreenGradient:
        // WrapperCommand pressureRedtoGreenGradientCommand =
        //     new RunCommand(
        //             () -> {
        var saturation2 = 0;
        // For every pixel
        for (var i = 93; i < 169; i++) {
          // Calculate the hue - hue is easier for rainbows because the color
          // shape is a circle so only one value needs to precess
          // saturation2 =
          //     (pressureRedtoGreenGradientFirstPixelSaturation + (i * 255 / 169))
          //         % 255;
          // Set the value

          int testS = 255;
          int testV = 255;

          ledBuffer.setHSV(i, outputHue, testV, pressureRedtoGreenGradientFirstPixelSaturation);

          if (pressureRedtoGreenGradientFirstPixelSaturation < 4) {
            brightening = true;
          } else if (pressureRedtoGreenGradientFirstPixelSaturation > 220) brightening = false;
        }
        if (brightening) {
          pressureRedtoGreenGradientFirstPixelSaturation += 3;
        } else {
          pressureRedtoGreenGradientFirstPixelSaturation -= 3;
        }
        // SmartDashboard.putBoolean("Brightening", brightening);
        // SmartDashboard.putNumber("Hue", outputHue);
        // SmartDashboard.putNumber(
        //     "Saturation", pressureRedtoGreenGradientFirstPixelSaturation);

        // System.out.println(rainbowFirstPixelHue);
        // Check bounds
        // pressureRedtoGreenGradientFirstPixelSaturation %= 255;

        leds.setData(ledBuffer);
        //             },
        //             this)
        //         .withName("Pressure Red to Green Gradient Lights");

        // return pressureRedtoGreenGradientCommand;
        // pressureRedtoGreenGradientCommand.schedule();
        // System.out.println("Pressure Red to Green Gradient!");

    }
  }

  private void pinkPulse() {

    int randomLED = (int) (Math.random() * ledBuffer.getLength());

    int r = 255;
    int g = 220;
    int b = 255;
    while (g > 0) {}
  }

  public Command multipleLightCommands(LEDState one, LEDState two) {
    return new RunCommand(
        () -> {
          getLEDStateCommand(one);
          getLEDStateCommand(two);
        });
  }

  private void convertHSV() {
    inputPSI = PneumaticsSystem.getInstance().psi;
    // SmartDashboard.putNumber("Pressure Proportional Input", inputPSI);
    outputHue = Math.round(((0 + slope) * (inputPSI - 3)));
    if (outputHue < 0) {
      outputHue = 0;
    }
    // SmartDashboard.putNumber("Pressure Proportional Hue", outputHue);
  }

  private Command setLights() {
    return new InstantCommand(() -> leds.setData(ledBuffer));
  }

  @Override
  public void periodic() {
    convertHSV();
    // System.out.println(slope);
    // System.out.println("test");
    // This method will be called once per scheduler run

  }
}
