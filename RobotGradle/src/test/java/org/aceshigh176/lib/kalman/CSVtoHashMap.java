package org.aceshigh176.lib.kalman;

import java.io.*;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.stream.Collectors;

public class CSVtoHashMap {

    public static ArrayList<String> splitCSVLine(String csvLine) {
        ArrayList<String> contents = new ArrayList<>();
        int start_idx = 0;
        while(start_idx < csvLine.length()) {
            int indexOfNextQuote = csvLine.indexOf('"', start_idx);
            int indexOfNextComma = csvLine.indexOf(',', start_idx);
            //If there is no comma, use the "end" of the string
            if(indexOfNextComma == -1) {
                indexOfNextComma = csvLine.length();
            }
            if(indexOfNextQuote == -1) {
                // There is no quote
                contents.add(csvLine.substring(start_idx, indexOfNextComma));
                start_idx = indexOfNextComma + 1;
            } else if(indexOfNextComma < indexOfNextQuote) {
                // The comma is before the quote, so this cell isn't quoted
                contents.add(csvLine.substring(start_idx, indexOfNextComma));
                start_idx = indexOfNextComma + 1;
            } else {
                // There are quotes in this cell
                int indexOfStartOfString = indexOfNextQuote + 1;
                int indexOfSecondQuote = csvLine.indexOf('"', indexOfStartOfString);
                contents.add(csvLine.substring(indexOfStartOfString, indexOfSecondQuote));
                start_idx = csvLine.indexOf(',', indexOfSecondQuote) + 1;
            }
        }
        contents = contents.stream().map(t -> t.trim()).collect(Collectors.toCollection(ArrayList::new));
        return contents;
    }

    public static HashMap<String, List<String>> convertCSVFileToHashMap(File csvFile) {
        HashMap<String, List<String>> jsonObject = new HashMap<>();
        try (BufferedReader br = new BufferedReader(new FileReader(csvFile))) {
            // First line should be all headers..
            String headerLine = br.readLine();
            ArrayList<String> headers = splitCSVLine(headerLine);

            for(String header: headers) {
                jsonObject.put(header, new ArrayList<>());
            }

            String line;
            while((line = br.readLine()) != null) {
                ArrayList<String> lineData = splitCSVLine(line);
                for(int i = 0; i < headers.size(); i++) {
                    jsonObject.get(headers.get(i)).add(lineData.get(i));
                }
            }
        } catch (FileNotFoundException e) {
            e.printStackTrace();
        } catch (IOException e) {
            e.printStackTrace();
        }
        return jsonObject;
    }
}

