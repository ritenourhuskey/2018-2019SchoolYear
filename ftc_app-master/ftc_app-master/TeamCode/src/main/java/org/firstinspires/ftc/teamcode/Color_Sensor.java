package org.firstinspires.ftc.teamcode;
import android.app.Activity;
import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

public class Color_Sensor extends LinearOpMode{

    ColorSensor color_sensor;

    /* These values are just filler values. Change all of these values to the values based on what you see the color sensor reports when testing.
     */
    int color = 20;
    int gold;
    int white;

    public void runOpMode(){
        // "name_of_sensor" should be replaced with the name given to the sensor on the robot controller phone
        color_sensor = hardwareMap.colorSensor.get("name_of_sensor");

        // alpha = total luminosity
        while (color_sensor.alpha() < color){
            // This keeps the LED on
            color_sensor.enableLed(true);

            if (color_sensor.argb() >= gold){
                // Knock off the golden mineral
            }
            else if (color_sensor.argb() >= white) {
                // Don't knock off white mineral
            }
            else {
                // Don't do anything
                //why
            }

        }
    }
}