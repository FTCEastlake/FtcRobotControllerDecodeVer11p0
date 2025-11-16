package org.firstinspires.ftc.teamcode.Burrrito;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BurrritoIntake {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private DcMotorEx _motorExIntake;

    public BurrritoIntake(LinearOpMode opMode) throws InterruptedException {
        _opMode = opMode;
        _hardwareMap = _opMode.hardwareMap;

        _motorExIntake = _hardwareMap.get(DcMotorEx.class, "motorIntake");
        _motorExIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        // This uses RUN_USING_ENCODER to be more accurate.
        _motorExIntake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExIntake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double ticksPerSecond) {
        _motorExIntake.setVelocity(ticksPerSecond);
    }

    public void setOn() {
        // Looks like 3000 ticks/sec is the fastest it will spin.
        // We can lower the velocity to save power.
        _motorExIntake.setVelocity(3000);
    }

    public void setOff() {
        _motorExIntake.setVelocity(0);
    }

}
