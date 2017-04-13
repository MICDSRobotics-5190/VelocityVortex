package org.firstinspires.ftc.teamcode.inputtracking;

import com.google.gson.stream.JsonWriter;
import com.qualcomm.robotcore.hardware.Gamepad;

import java.io.IOException;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.util.ArrayList;

/**
 * Created by BAbel on 4/12/2017.
 */

public class InputWriter {

    public void writeJson(OutputStream out, ArrayList<Input> inputs) throws IOException{
        JsonWriter writer = new JsonWriter(new OutputStreamWriter(out, "UTF-8"));
        writer.setIndent("    ");
        writeInputArray(writer, inputs);
        writer.close();
    }

    public void writeInputArray(JsonWriter writer, ArrayList<Input> inputs) throws IOException{
        writer.beginArray();
        for (Input input : inputs){
            writeInput(writer, input);
        }
        writer.endArray();
    }

    public void writeInput(JsonWriter writer, Input input) throws IOException{

        writer.beginObject();
        writer.name("left_stick").value(input.getLeftStickY());
        writer.name("right_stick").value(input.getRightStickY());
        //writer.name("").value() Other objects

        writer.name("time").value(input.getCurrentTime());

        writer.endObject();
    }


}
