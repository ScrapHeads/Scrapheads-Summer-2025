package org.firstinspires.ftc.teamcode.tuning;

import static org.firstinspires.ftc.teamcode.Constants.*;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Pose2d;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.vision.CameraParams;
import org.firstinspires.ftc.teamcode.vision.VisionProcessor;

@TeleOp(name = "Vision Calibration", group = "ScrapHeads")
public class VisionCalibration extends OpMode {
    private HuskyLens huskyLens;
    private VisionProcessor processor;
    private FtcDashboard dash;

    @Override
    public void init() {
        huskyLens = hardwareMap.get(HuskyLens.class, "huskylens");
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

        processor = new VisionProcessor(new CameraParams(
                CAMERA_FORWARD_OFFSET,
                CAMERA_LATERAL_OFFSET,
                CAMERA_VERTICAL_OFFSET,
                CAMERA_YAW_OFFSET
        ));

        dash = FtcDashboard.getInstance();

        telemetry.addLine("Vision Calibration Initialized");
        telemetry.update();
    }

    @Override
    public void loop() {
        HuskyLens.Block[] blocks = huskyLens.blocks();
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Tag Count", blocks.length);

        if (blocks.length > 0) {
            // Just take the first block for testing
            Pose2d pose = processor.blockToRobotPose(blocks[0]);

            if (pose != null) {
                double forward = pose.position.x; // +X forward
                double left = pose.position.y;    // +Y left
                double headingDeg = Math.toDegrees(pose.heading.toDouble());

                telemetry.addData("Forward (in)", forward);
                telemetry.addData("Left (in)", left);
                telemetry.addData("Heading (deg)", headingDeg);

                packet.put("Forward (in)", forward);
                packet.put("Left (in)", left);
                packet.put("Heading (deg)", headingDeg);
            }
        } else {
            telemetry.addLine("No tags detected");
        }

        telemetry.update();
        dash.sendTelemetryPacket(packet);
    }
}
