package org.firstinspires.ftc.teamcode.TestBench;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.ColorSensorV3;

@Disabled
@Configurable
@TeleOp(name = "TestBenchColorSensor")
public class TestBenchColorSensor extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    I2C bus3: "colorSensorV3"  (REV Color Sensor V3)

    private ColorSensorV3 _colorSensor = null;
    private ColorSensorV3.DetectedColor _detectedColor;

    private HardwareMap _hardwareMap;
    private TelemetryManager _telemetryM;
    private float _gain;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        _gain = 1.0f;
        _colorSensor.setGain(_gain);

        while (!isStopRequested()) {

            if (gamepad1.leftBumperWasPressed())
            {
                _gain -= 0.1f;
                if (_gain < 0.5f)
                    _gain = 0.5f;
                _colorSensor.setGain(_gain);
            }

            if (gamepad1.rightBumperWasPressed())
            {
                _gain += 0.1f;
                if (_gain > 10.0f)
                    _gain = 10.0f;
                _colorSensor.setGain(_gain);
            }


            _telemetryM.addData("Gain = ", _gain);
            telemetry.addData("Gain = ", _gain);
            _colorSensor.displayColorValues();
            //_detectedColor = _colorSensor.getDetectedColor();

            _telemetryM.update();
            telemetry.update();

        }

    }

    private void initRobot() throws InterruptedException {

        _hardwareMap = hardwareMap;
        _telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        _colorSensor = new ColorSensorV3(this, _telemetryM, telemetry);
    }



}