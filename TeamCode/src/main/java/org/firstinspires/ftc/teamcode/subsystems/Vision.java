package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;
import static org.firstinspires.ftc.teamcode.Constants.dashboard;
import static org.firstinspires.ftc.teamcode.Constants.hm;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.command.Subsystem;
import com.qualcomm.hardware.dfrobot.HuskyLens;

import org.firstinspires.ftc.robotcore.internal.system.Deadline;

import java.util.concurrent.TimeUnit;

public class Vision implements Subsystem {
    private HuskyLens huskyLens;
    private final int READ_PERIOD = 1;

    int t = 1;


    Deadline rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);


    public Vision() {
        huskyLens = hm.get(HuskyLens.class, "huskylens");
        /*
         * This sample rate limits the reads solely to allow a user time to observe
         * what is happening on the Driver Station telemetry.  Typical applications
         * would not likely rate limit.
         */

        // Immediately expire so that the first time through we'll do the read.
        rateLimit.expire();

        /*
         * Basic check to see if the device is alive and communicating.  This is not
         * technically necessary here as the HuskyLens class does this in its
         * doInitialization() method which is called when the device is pulled out of
         * the hardware map.  However, sometimes it's unclear why a device reports as
         * failing on initialization.  In the case of this device, it's because the
         * call to knock() failed.
         */
        TelemetryPacket packet = new TelemetryPacket();

        if (!huskyLens.knock()) {
            packet.put(">>", "Problem communicating with " + huskyLens.getDeviceName());
        } else {
            packet.put(">>", "Press start to continue");
        }
        dashboard.sendTelemetryPacket(packet);

        /*
         * The device uses the concept of an algorithm to determine what types of
         * objects it will look for and/or what mode it is in.  The algorithm may be
         * selected using the scroll wheel on the device, or via software as shown in
         * the call to selectAlgorithm().
         *
         * The SDK itself does not assume that the user wants a particular algorithm on
         * startup, and hence does not set an algorithm.
         *
         * Users, should, in general, explicitly choose the algorithm they want to use
         * within the OpMode by calling selectAlgorithm() and passing it one of the values
         * found in the enumeration HuskyLens.Algorithm.
         *
         * Other algorithm choices for FTC might be: OBJECT_RECOGNITION, COLOR_RECOGNITION or OBJECT_CLASSIFICATION.
         */
        huskyLens.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);

//        telemetry.update();
    }

    @Override
    public void periodic() {
        detectObject();
    }

    public void detectObject() {
        if (!rateLimit.hasExpired()) {
            detectObject();
        }
        rateLimit.reset();

        /*
         * All algorithms, except for LINE_TRACKING, return a list of Blocks where a
         * Block represents the outline of a recognized object along with its ID number.
         * ID numbers allow you to identify what the device saw.  See the HuskyLens documentation
         * referenced in the header comment above for more information on IDs and how to
         * assign them to objects.
         *
         * Returns an empty array if no objects are seen.
         */
        TelemetryPacket packet = new TelemetryPacket();
        packet.put("here", 0);
        dashboard.sendTelemetryPacket(packet);

        HuskyLens.Block[] blocks = huskyLens.blocks();
        packet.put("Block count", blocks.length);
        // For loop is set up as
        // INITIALIZATION; CONDITION; INCREMENT
        // Initialization: Runs once, at the very beginning
        // CONDITION: Checked before every loop iteration
        // INCREMENT: Runs after each loop iteration
        for (int i = 0; i < blocks.length; i++) {
            packet.put("Block", blocks[i].toString());
            packet.put("x", blocks[i].x);
            packet.put("y", blocks[i].y);

            /*
             * Here inside the FOR loop, you could save or evaluate specific info for the currently recognized Bounding Box:
             * - blocks[i].width and blocks[i].height   (size of box, in pixels)
             * - blocks[i].left and blocks[i].top       (edges of box)
             * - blocks[i].x and blocks[i].y            (center location)
             * - blocks[i].id                           (Color ID)
             *
             * These values have Java type int (integer).
             */
            t++;
            packet.put("Cycles", t);
            dashboard.sendTelemetryPacket(packet);
        }
        t = 0;
        dashboard.sendTelemetryPacket(packet);
        packet.clearLines();

    }
}
