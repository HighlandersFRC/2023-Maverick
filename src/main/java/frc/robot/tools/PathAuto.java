// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.tools;

import java.io.File;
import java.io.FileReader;

import org.json.JSONArray;
import org.json.JSONObject;
import org.json.JSONTokener;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/** Add your docs here. */
public abstract class PathAuto extends SequentialCommandGroup {
    public abstract JSONArray getStartingPath();
    public JSONObject getPathJSONObject(String path){
        File pathFile;
        JSONObject pathRead;
        try {
            pathFile = new File(path);
            FileReader scanner = new FileReader(pathFile);
            pathRead = new JSONObject(new JSONTokener(scanner));
        } catch(Exception e) {
            System.out.println("ERROR WITH PATH FILE " + e);
            pathRead = null;
        }
        return pathRead;
    }
    public JSONArray getPathPoints(String path){
        JSONObject pathRead = getPathJSONObject(path);
        JSONArray pathArray;
        try{
            pathArray = (JSONArray) pathRead.getJSONArray("sampled_points");
        } catch (Exception e){
            pathArray = null;
            System.out.println("JSONArray was not found at key \"sampled_points\" with error code:\n"+e.getMessage());
        }
        return pathArray;
    }
}
