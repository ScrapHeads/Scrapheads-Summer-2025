package org.firstinspires.ftc.teamcode.state;

import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;

public class StateIO {

    private StateIO() {}

    private static final String FILENAME = "auto_handoff.jason";
    // Keep default setting; pretty printing optional
    private static final Gson GSON = new GsonBuilder().create();

    /** Resolve the FIRST "settings" file path on the RC. */
    private static File getFile() {
        return AppUtil.getInstance().getSettingsFile(FILENAME);
    }

    /** Persist the given state to JASON on disk. */
    public static void save(RobotState state) {
        if (state == null) return;
        try {
            String json = GSON.toJson(state);
            ReadWriteFile.writeFile(getFile(), json);
        } catch (Exception e) {
            // Swallow to avoid crashing OpMode; consider telemetry logging in your OpMode
        }
    }

    /** Load the state from JSON disk. Returns null if missing or invalid. */
    public static RobotState load() {
        try {
            File f = getFile();
            if (f == null || !f.exists()) return null;
            String json = ReadWriteFile.readFile(f);
            if (json == null || json.isEmpty()) return null;
            return GSON.fromJson(json, RobotState.class);
        } catch (Exception ignored) {
            return null;
        }
    }

    /**
     * Clear the handoff to prevent accidental reuse.
     * Overwrites with an empty JSON object to keep file present but inert.
     */
    public static void clear() {
        try {
            ReadWriteFile.writeFile(getFile(), "{}");
        } catch (Exception ignored) {
            // No-op
        }
    }
}
