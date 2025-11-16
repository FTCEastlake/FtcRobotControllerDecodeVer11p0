package org.firstinspires.ftc.teamcode.Burrrito;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BurrritoBelt {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;
    private DcMotorEx _motorExBelt;

    public BurrritoBelt(LinearOpMode opMode) throws InterruptedException {
        _opMode = opMode;
        _hardwareMap = _opMode.hardwareMap;

        _motorExBelt = _hardwareMap.get(DcMotorEx.class, "motorBelt");
        _motorExBelt.setDirection(DcMotorSimple.Direction.REVERSE);
        // This uses RUN_USING_ENCODER to be more accurate.
        _motorExBelt.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        _motorExBelt.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setVelocity(double ticksPerSecond) {
        _motorExBelt.setVelocity(ticksPerSecond);
    }

    public void setOn() {
        // TODO: test out velocity.
        // We can lower the velocity to save power.
        _motorExBelt.setVelocity(1000);
    }

    public void setOff() {
        _motorExBelt.setVelocity(0);
    }

}
