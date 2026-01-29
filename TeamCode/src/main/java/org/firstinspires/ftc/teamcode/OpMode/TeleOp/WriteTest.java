package org.firstinspires.ftc.teamcode.OpMode.TeleOp;
import android.content.Context;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware.Sensors;
import org.json.JSONObject;

import java.io.File;
import java.io.FileWriter;

@TeleOp(name = "Write Test")
public class WriteTest extends OpMode {
    Sensors sensors;


    @Override
    public void init() {
        writeJson(hardwareMap.appContext);
    }

    @Override
    public void loop() {

    }

    public void writeJson(Context context){
        try {
            JSONObject json = new JSONObject();
            json.put("lastX", "sugi");
            json.put("lastY", "sugi de doua ori");
            json.put("lastHeading", "sugi si o a3-a oara");
            File file = new File(context.getFilesDir(), "config.json");
            FileWriter writer = new FileWriter(file);
            writer.write(json.toString());
            writer.close();

        } catch (Exception e){
            e.printStackTrace();
        }
    }
}
