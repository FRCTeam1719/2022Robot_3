// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.AutoCloseable;


import java.lang.Object;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.simulation.AddressableLEDSim;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.revrobotics.* ;


// import com.qualcomm.robotcore.eventloop.opmode.Disabled;
// import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
// import com.qualcomm.robotcore.eventloop.opmode.OpMode;
// import com.qualcomm.robotcore.hardware.HardwareMap;

// import org.firstinspires.ftc.robotcore.external.Telemetry;
// import org.firstinspires.ftc.robotcore.internal.system.Deadline;

// import java.util.concurrent.TimeUnit;

// import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.*;
// import static com.qualcomm.hardware.rev.RevBlinkinLedDriver.BlinkinPattern.*;

public class LedSubsystem extends SubsystemBase {
    /** Creates a new ArmSubsystem. */

    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;
    private int move;
    public LedSubsystem() {
        this.led = new AddressableLED(0);
        this.ledBuffer = new AddressableLEDBuffer(60);
        this.led.setLength(ledBuffer.getLength());
        this.led.setData(ledBuffer);
        this.led.start();
        this.move = 0;
    }
    
    public void LIME(){
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.ledBuffer.setRGB(i, 255, 87, 51);
         }}
    public void LBLUE(){
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.ledBuffer.setRGB(i, 3, 202, 252);
         }}
    public void YELLOW(){
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.ledBuffer.setRGB(i, 252, 252, 3);
         }}
    public void RED(){
        for (var i = 0; i < this.ledBuffer.getLength(); i++) {
            // Sets the specified LED to the RGB values for red
            this.ledBuffer.setRGB(i, 250, 0, 0);
         }}
    public void RAINBOW() {
    // For every pixel
    for (var i = 0; i < this.ledBuffer.getLength(); i++) {
      // Calculate the hue - hue is easier for rainbows because the color
      // shape is a circle so only one value needs to precess
    
      // Set the value
      this.ledBuffer.setHSV(i,( i + this.move) % 180, 255, 128);
    }
    // Increase by move to make the rainbow "move"

  }

         
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        this.LIME();

        this.led.setData(ledBuffer);
        this.move+=1;
    }

}



