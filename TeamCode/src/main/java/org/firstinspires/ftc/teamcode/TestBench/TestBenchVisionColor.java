package org.firstinspires.ftc.teamcode.TestBench;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Common.VisionColor;

@Disabled
@Configurable
@TeleOp(name = "TestBenchVisionColor")
public class TestBenchVisionColor extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    Webcam: "WebcamColor"  (REV Color Sensor V3)

    private VisionColor _colorSensor = null;

    private HardwareMap _hardwareMap;
    private TelemetryManager _telemetryM;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        while (!isStopRequested()) {

            _colorSensor.DisplayColorResults();
            _telemetryM.update();
            telemetry.update();

            sleep(20);

        }

    }

    private void initRobot() throws InterruptedException {

        _hardwareMap = hardwareMap;
        _telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // Pass in what percentage of the whole image you want to use to detect color.
        // You're defining a square round the center of the image to process for color.
        // 1.0 = use all of image to detect color.
        // 0.5 = use a square that's 50% of the image (around center) to detect color.
        // 0.1 = use a square that's 10% of the image (around center) to detect color.
        double centerAreaPercentage = 0.5;
        _colorSensor = new VisionColor(this, _telemetryM, telemetry, centerAreaPercentage);
    }



}