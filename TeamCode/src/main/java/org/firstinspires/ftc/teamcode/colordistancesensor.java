package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp
public class colordistancesensor extends LinearOpMode {

    private ColorSensor imu_REV_ColorRangeSensor;
    private DistanceSensor sensorRange;

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        int gain;
        NormalizedRGBA normalizedColors;
        int color;
        float hue;
        float saturation;
        float value;

        imu_REV_ColorRangeSensor = hardwareMap.get(ColorSensor.class, "imu");
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");

        // This op mode demonstrates the color and distance features of the REV sensor.
        gain = 2;
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        telemetry.addData(">>", "Press start to continue");
        telemetry.update();
        telemetry.addData("Color Distance Example", "Press start to continue...");
        telemetry.update();
        waitForStart();
        if (opModeIsActive()) {
            // Put run blocks here.
            while (opModeIsActive()) {
                // Adjust the gain.
                if (gamepad1.a) {
                    gain += 0.005;
                } else if (gamepad1.b && gain >= 1.005) {
                    gain += -0.005;
                }
                ((NormalizedColorSensor) imu_REV_ColorRangeSensor).setGain(gain);
                // Read color from the sensor.
                normalizedColors = ((NormalizedColorSensor) imu_REV_ColorRangeSensor).getNormalizedColors();
                telemetry.addData("Red", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.red, 3)));
                telemetry.addData("Green", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.green, 3)));
                telemetry.addData("Blue", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.blue, 3)));
                // Convert RGB values to Hue, Saturation, and Value.
                // See https://en.wikipedia.org/wiki/HSL_and_HSV for details on HSV color model.
                color = normalizedColors.toColor();
                hue = JavaUtil.colorToHue(color);
                saturation = JavaUtil.colorToSaturation(color);
                value = JavaUtil.colorToValue(color);
                telemetry.addLine("maximum distance: 15cm");
                telemetry.addData("range", String.format("%.01f mm", sensorRange.getDistance(DistanceUnit.MM)));
                telemetry.addData("range", String.format("%.01f cm", sensorRange.getDistance(DistanceUnit.CM)));
                telemetry.addData("range", String.format("%.01f m", sensorRange.getDistance(DistanceUnit.METER)));
                telemetry.addData("Hue", Double.parseDouble(JavaUtil.formatNumber(hue, 0)));
                telemetry.addData("Saturation", Double.parseDouble(JavaUtil.formatNumber(saturation, 3)));
                telemetry.addData("Value", Double.parseDouble(JavaUtil.formatNumber(value, 3)));
                telemetry.addData("Alpha", Double.parseDouble(JavaUtil.formatNumber(normalizedColors.alpha, 3)));

                // Rev2mDistanceSensor specific methods.
                telemetry.addData("did time out", Boolean.toString(sensorTimeOfFlight.didTimeoutOccur()));
                // Show the color on the Robot Controller screen.
                JavaUtil.showColor(hardwareMap.appContext, color);
                // Use hue to determine if it's red, green, blue, etc..
                if (hue < 30) {
                    telemetry.addData("Color", "Red");
                } else if (hue < 60) {
                    telemetry.addData("Color", "Orange");
                } else if (hue < 90) {
                    telemetry.addData("Color", "Yellow");
                } else if (hue < 150) {
                    telemetry.addData("Color", "Green");
                } else if (hue < 225) {
                    telemetry.addData("Color", "Blue");

                } else if (hue < 350) {
                    telemetry.addData("Color", "purple");
                } else {
                    telemetry.addData("Color", "Red");
                }
                // Check to see if it might be black or white.
                if (saturation < 0.2) {
                    telemetry.addData("Check Sat", "Is surface white?");
                }
                telemetry.update();
                if (value < 0.16) {
                    telemetry.addData("Check Val", "Is surface black?");
                }
            }
            // Show white on the Robot Controller screen.
            JavaUtil.showColor(hardwareMap.appContext, Color.parseColor("white"));
        }
    }
}