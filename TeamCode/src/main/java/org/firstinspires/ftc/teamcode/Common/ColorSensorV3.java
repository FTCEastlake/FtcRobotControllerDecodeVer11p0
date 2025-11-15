package org.firstinspires.ftc.teamcode.Common;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorV3 {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private Telemetry _telemetry;
    private NormalizedColorSensor _colorSensor;

    public enum DetectedColor {
        RED,
        GREEN,
        BLUE,
        YELLOW,
        UNKNOWN
    }

    public ColorSensorV3(LinearOpMode opMode, Telemetry telemetry) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;
        _telemetry = telemetry;
        init();
    }

    private void init()
    {
        _colorSensor = _hardwareMap.get(NormalizedColorSensor.class, "colorSensorV3");
    }

    public void setGain(float gain) {
        _colorSensor.setGain(gain);
    }

    float _redThreshold_Red = 1.0F, _redThreshold_Green = 0.0F, _redThreshold_Blue = 0.0F;
    public void setDetectedThresholdsRed(float normRed, float normalGreen, float normalBlue)
    {
        _redThreshold_Red = normRed;
        _redThreshold_Green = normalGreen;
        _redThreshold_Blue = normalBlue;
    }

    float _greenThreshold_Red = 0.0F, _greenThreshold_Green = 1.0F, _greenThreshold_Blue = 0.0F;
    public void setDetectedThresholdsGreen(float normRed, float normalGreen, float normalBlue)
    {
        _greenThreshold_Red = normRed;
        _greenThreshold_Green = normalGreen;
        _greenThreshold_Blue = normalBlue;
    }

    float _blueThreshold_Red = 0.0F, _blueThreshold_Green = 0.0F, _blueThreshold_Blue = 1.0F;
    public void setDetectedThresholdsBlue(float normRed, float normalGreen, float normalBlue)
    {
        _blueThreshold_Red = normRed;
        _blueThreshold_Green = normalGreen;
        _blueThreshold_Blue = normalBlue;
    }

    public DetectedColor getDetectedColor() {

        NormalizedRGBA colors = _colorSensor.getNormalizedColors();

        float normalRed = colors.red / colors.alpha;
        float normalGreen = colors.green / colors.alpha;
        float normalBlue = colors.blue / colors.alpha;

        // watch youtube: https://www.youtube.com/watch?v=pyIeknIcT8M
        // How to calibrate colors:
        // 1) Place the red object at the detection distance.
        // 2) Call setGain() greater than 1.0 until the red value is greater than 0.35 when calling displayColorValues().
        // 3) Call setDetectedThresholdsRed(...) passing in the normRed, normalGreen and normalBlue values.
        // 4) Repeat steps 1-3 for blue and green colors.
        // NOTE: We only want to setGain() to a high enough value so that the detection of the main colors are greater than 0.35.

        if (normalRed > _redThreshold_Red && normalGreen < _redThreshold_Green && normalBlue < _redThreshold_Blue)
            return DetectedColor.RED;
        else if (normalGreen > _greenThreshold_Green && normalRed < _greenThreshold_Red && normalBlue < _greenThreshold_Blue)
            return DetectedColor.GREEN;
        else if (normalBlue > _blueThreshold_Blue && normalRed < _blueThreshold_Red && normalGreen < _blueThreshold_Green)
            return DetectedColor.BLUE;
        else
            return DetectedColor.UNKNOWN;
    }

    public void displayColorValues() {
        // Returns red, green, blue, alpha
        NormalizedRGBA colors = _colorSensor.getNormalizedColors();

        float normalRed = colors.red / colors.alpha;
        float normalGreen = colors.green / colors.alpha;
        float normalBlue = colors.blue / colors.alpha;

        _telemetry.addData("red", normalRed);
        _telemetry.addData("green", normalGreen);
        _telemetry.addData("blue", normalBlue);
        //_telemetry.update();
    }

}
