package org.firstinspires.ftc.teamcode.OpMode.TeleOp;
import android.content.Context;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Util.Wrapper.TelemetryUtil;
import org.json.JSONObject;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;

@TeleOp(name = "Read Test")
public class ReadJson extends OpMode {
    @Override
    public void init() {
        TelemetryUtil.setup();
        JSONObject json = readJson(hardwareMap.appContext);
        String lastX = null;
        String lastY = null;
        String lastHeading = null;
        if (json != null) {
            try {
                lastX = json.getString("lastX");
                lastY = json.getString("lastY");
                lastHeading = json.getString("lastHeading");
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
        TelemetryUtil.packet.put("LastY", lastY);
        TelemetryUtil.packet.put("Last Heading", lastHeading);
        TelemetryUtil.sendTelemetry();

    }

    @Override
    public void loop() {

    }

    public JSONObject readJson(Context context) {
        try {
            File file = new File(context.getFilesDir(), "config.json");

            if (!file.exists()) {
                return null;
            }

            BufferedReader reader = new BufferedReader(new FileReader(file));
            StringBuilder builder = new StringBuilder();
            String line;

            while ((line = reader.readLine()) != null) {
                builder.append(line);
            }
            reader.close();

            return new JSONObject(builder.toString());

        } catch (Exception e) {
            e.printStackTrace();
            return null;
        }
    }


}
