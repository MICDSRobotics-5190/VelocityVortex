package org.firstinspires.ftc.teamcode.inputtracking;

import android.util.JsonReader;

import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.util.ArrayList;

/**
 * Created by BAbel on 4/12/2017.
 */

public class InputReader {

    public InputReader(){}

    public ArrayList<Input> readJson(InputStream in) throws IOException {
        JsonReader reader = new JsonReader(new InputStreamReader(in, "UTF-8"));

        try{
            return readInputArray(reader);
        } finally {
            reader.close();
        }

    }

    public ArrayList<Input> readInputArray(JsonReader reader) throws IOException {
        ArrayList<Input> inputs = new ArrayList<Input>();

        reader.beginArray();
        while (reader.hasNext()){
            inputs.add(readInput(reader));
        }
        reader.endArray();

        return inputs;
    }

    public Input readInput(JsonReader reader) throws IOException {
        Input input = new Input();

        reader.beginObject();
        while(reader.hasNext()){
            String name = reader.nextName();
            switch (name){
                case "time": input.setCurrentTime(reader.nextDouble());
                    break;
                case "left_stick": input.setLeftStickY(reader.nextDouble());
                    break;
                case "right_stick": input.setRightStickY(reader.nextDouble());
                    break;
                default: reader.skipValue();
                    break;
            }
        }
        reader.endObject();

        return input;
    }

}
