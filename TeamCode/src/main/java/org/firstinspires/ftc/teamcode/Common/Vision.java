package org.firstinspires.ftc.teamcode.Common;

import android.util.Size;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagPoseFtc;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.opencv.ImageRegion;
import org.firstinspires.ftc.vision.opencv.PredominantColorProcessor;

import java.util.List;

public class Vision {

    private LinearOpMode _opMode;
    private HardwareMap _hardwareMap;

    private VisionPortal _visionPortal = null;               // Used to manage the video source.
    private AprilTagProcessor _aprilTag = null;              // Used for managing the AprilTag detection process.

    public Vision(LinearOpMode opMode) throws InterruptedException{
        _opMode = opMode;
        _hardwareMap = opMode.hardwareMap;

        boolean includeAprilTagDetection =  true;
        init(includeAprilTagDetection);
    }

    private void init(boolean enableAprilTagDetection)
    {
        boolean useGoBuildaCamera  = true;
        if (enableAprilTagDetection)
        {
            // Create the AprilTag processor by using a builder.
            if (useGoBuildaCamera)
                _aprilTag = new AprilTagProcessor.Builder()
                        .setLensIntrinsics(481.985, 481.985, 334.203, 241.948)
                        .build();
            else
                _aprilTag = new AprilTagProcessor.Builder().build();

            // Adjust Image Decimation to trade-off detection-range for detection-rate.
            // e.g. Some typical detection data using a Logitech C920 WebCam
            // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
            // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
            // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
            // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
            // Note: Decimation can be changed on-the-fly to adapt during a match.
            _aprilTag.setDecimation(2);
        }



        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(_hardwareMap.get(WebcamName.class, "Webcam Front"));
        if (useGoBuildaCamera)
            builder.setCameraResolution(new Size(640,480));
        if (enableAprilTagDetection)
        {
            builder.addProcessor(_aprilTag);
        }
        _visionPortal = builder.build();
    }

    public Pose getFirstDetectedTag() {

        List<AprilTagDetection> currentDetections = _aprilTag.getDetections();
        if (currentDetections.isEmpty())
            return null;

        AprilTagPoseFtc ftcPose = currentDetections.get(0).ftcPose;

        // Note1: ftcPose.y is pedroPose.x which represents front (+) and back (-)
        // Note2: ftcPose.x is pedroPose.y which represents left (+) and right (-)
        // Note2: ftcPose.yaw is in degrees. Negative is consistent with perdo Pose counter clockwise (+)
        return new Pose(ftcPose.y, -ftcPose.x, -ftcPose.yaw);
    }

}
