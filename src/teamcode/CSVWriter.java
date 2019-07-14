package teamcode;

import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

public class CSVWriter
{
    FileWriter csvWriter;

    public CSVWriter()
    {
        try
        {
            csvWriter = new FileWriter("/Users/kalee1/Desktop/PurePursuit.csv");
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

    }

    public void addRow(List<String> rowData)
    {
//        for (String item : rowData )
        {
            try
            {
                csvWriter.append(String.join(",", rowData));
            }
            catch (IOException e)
            {
                e.printStackTrace();
            }

        }
        try
        {
            csvWriter.append("\n");
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

        try
        {
            csvWriter.flush();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }
    }

    public void close()
    {
        try
        {
            csvWriter.flush();
            csvWriter.close();
        }
        catch (IOException e)
        {
            e.printStackTrace();
        }

    }

}
