
package org.firstinspires.ftc.teamcode.TestBench;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Disabled
@Configurable
@TeleOp(name = "TestBenchMotor")
public class TestBenchMotor extends LinearOpMode {

    //**************************************************************
    // Control Hub:
    //    Motor port0: "motor0"  (GoBILDA 5202/3/4 series)

    // Declare motors
    private DcMotor _motor0 = null;

    private HardwareMap _hardwareMap;
    //private ColorSensorV3 _colorSensor;


    @Override
    public void runOpMode() throws InterruptedException {

        initRobot();

        waitForStart();

        double power = 0.0;

        while (!isStopRequested()) {

            if (gamepad1.left_bumper)
                power = gamepad1.left_stick_y * 1.0;
            else
                power = gamepad1.left_stick_y * 0.25;

            _motor0.setPower(power);

            telemetry.addData("motor0 power", power);
            telemetry.update();

        }

    }

    private void initRobot() throws InterruptedException {

        _hardwareMap = hardwareMap;

        // Make sure your ID's match your configuration
        _motor0 = _hardwareMap.get(DcMotor.class, "motor0");
        //_colorSensor = new ColorSensorV3(this, telemetry);
    }



}