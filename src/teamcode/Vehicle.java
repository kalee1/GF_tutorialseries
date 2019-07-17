package teamcode;

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Path Following

// Vehicle class

import processing.core.PVector;
import sbfServer.Range;

import java.util.ArrayList;
import java.util.List;

public class Vehicle
{

    // All the usual stuff
    PVector position;
    PVector velocity;
    PVector acceleration;
    float maxforce;    // Maximum steering force
    float maxspeed;    // Maximum speed

    float heading;
    float angularVelocity;
    float angularAcceleration;


    boolean arriving = false;
    int currentPathSeg = 0;
    int numPathPoints = 0;
    float closingDist = (float)1000000.0;
    CSVWriter logFile;
    List<String> rowData;

    // Constructor initialize all values
    public Vehicle(PVector l, float heading, float ms, float mf)
    {
        position = l.copy();
        maxspeed = ms;
        maxforce = mf;
        acceleration = new PVector(0, 0);
        velocity = new PVector(-maxspeed, maxspeed);

        logFile = new CSVWriter();
        rowData = new ArrayList<String>();

        rowData.add(String.valueOf("position.x"));
        rowData.add(String.valueOf("position.y"));
        rowData.add(String.valueOf("position.heading"));

        rowData.add(String.valueOf("predictedPosition.x"));
        rowData.add(String.valueOf("predictedPosition.y"));
        rowData.add("closingDistance");

        rowData.add(String.valueOf("normalPoint.x"));
        rowData.add(String.valueOf("normalPoint.y"));

        rowData.add(String.valueOf("a.x"));
        rowData.add(String.valueOf("a.y"));

        rowData.add(String.valueOf("b.x"));
        rowData.add(String.valueOf("b.y"));

        rowData.add(String.valueOf("targetVector.x"));
        rowData.add(String.valueOf("targetVector.y"));
//        logFile.addRow(rowData);
        rowData.clear();

    }

    public float getX()
    {
        return position.x;
    }

    public float getY()
    {
        return position.y;
    }

    public float getHeading()
    {
        return (float) Math.toRadians(heading);
    }

    // This function implements Craig Reynolds' path following algorithm
    // http://www.red3d.com/cwr/steer/PathFollow.html
    //
    // 1. Check vehicle's future location
    // 2. Is that future location on the path?  (if yes, do nothing)
    // 3. Find closest point on the path to the future location (use dot product and project to path segment)
    // 4. Move that projected point along the path a little bit -- this becomes the target
    // 5. Seek the target
    //
    public void follow(Path thePath)
    {
        numPathPoints = thePath.points.size();

        // Based on which segment I'm on, I need to set a flag so that the seek method will know that is has to
        // arrive instead of just seeking.
        if (currentPathSeg == (numPathPoints - 2))
        {
            arriving = true;
        }

        PVector targetVector = null;
        PVector a = null;
        PVector b = null;
        PVector normalPoint = null;

        // Pull out 2 points to form the current segment of the path
        a = thePath.points.get(currentPathSeg);
        b = thePath.points.get(currentPathSeg + 1);

        closingDist = PVector.dist(position, b);

        //
        // Step 1
        //
        // Predict position small amt ahead by adding velocity to current position
        PVector predictedPosition = null;
        if ( closingDist > thePath.pointRadius )
        {
            PVector velocityCopy = velocity.copy();
            velocityCopy.setMag(5);

            // Add the scaled velocity vector to your current position to get the predicted future position
            predictedPosition = PVector.add(position, velocityCopy);
        }
        else if ( arriving )
        {
            predictedPosition = b.copy();
        }
        else
        {
            predictedPosition = position.copy();
        }
        rowData.add(String.valueOf(position.x));
        rowData.add(String.valueOf(position.y));
        rowData.add(String.valueOf(position.heading()));

        rowData.add(String.valueOf(predictedPosition.x));
        rowData.add(String.valueOf(predictedPosition.y));

        //
        // Step 2
        //
        // Now we must find the normal to the path from the predicted position.



        rowData.add(String.valueOf(closingDist));

        normalPoint = getNormalPoint(predictedPosition, a, b);

        if (PVector.dist(normalPoint, b) > PVector.dist(a, b))
        {
            normalPoint = a.copy();
        }
        else if (PVector.dist(a, b) < PVector.dist(a, normalPoint))
        {
            normalPoint = b.copy();
        }

        rowData.add(String.valueOf(normalPoint.x));
        rowData.add(String.valueOf(normalPoint.y));

        //
        // Step 4
        //
        // Move that projected point along the path a little bit -- this becomes the target
        //

        // Initially, set the target to the same as the normal
        targetVector = normalPoint.copy();

        // Look at the direction of the line segment so we can seek a little bit ahead of the normal
        PVector pathDir = PVector.sub(b, a);
        pathDir.setMag((float)(thePath.pointRadius));

        if (!arriving )
        {
            targetVector.add(pathDir);

        }
        else if (closingDist > thePath.pointRadius)
        {
            targetVector.add(pathDir);
        }


        rowData.add(String.valueOf(a.x));
        rowData.add(String.valueOf(a.y));

        rowData.add(String.valueOf(b.x));
        rowData.add(String.valueOf(b.y));

        rowData.add(String.valueOf(targetVector.x));
        rowData.add(String.valueOf(targetVector.y));
        logFile.addRow(rowData);


//        System.out.printf("Car    Pt: %.2f, %.2f ", position.x, position.y);
//        System.out.println("Current Path Segment: " + currentPathSeg );
//        System.out.println();

        seek(targetVector, thePath.headings.get(currentPathSeg), thePath.maxspeeds.get(currentPathSeg));

        if ( !arriving && PVector.dist(b, normalPoint) < thePath.pointRadius )
        {
            currentPathSeg++;
        }
        rowData.clear();
    }


    // A function to get the normal point from a point (p) to a line segment (a-b)
    // This function could be optimized to make fewer new Vector objects
    PVector getNormalPoint(PVector p, PVector a, PVector b)
    {
        // Vector from a to p
        PVector ap = PVector.sub(p, a);
        // Vector from a to b
        PVector ab = PVector.sub(b, a);
        ab.normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product
        ab.mult(ap.dot(ab));
        PVector normalPoint = PVector.add(a, ab);
        return normalPoint;
    }

    // Method to update position
    public void update()
    {
        // Update velocity
        velocity.add(acceleration);
        // Limit speed
        velocity.limit(maxspeed);
        position.add(velocity);
        // Reset accelertion to 0 each cycle
        acceleration.mult(0);

        angularVelocity += angularAcceleration;
        angularVelocity = Range.clip(angularVelocity, -10, 10);
        heading += angularVelocity;
    }

    // A method that calculates and applies a steering force towards a target
    // STEER = DESIRED MINUS VELOCITY
    void seek(PVector target, float hdg, float maxspd)
    {
        PVector neededVelocity = PVector.sub(target, position);  // A vector pointing from the position to the target

        //System.out.println(desired);

        float v = neededVelocity.mag();
//        System.out.printf("neededvelocity mag: %.2f\n", v);
        // If the magnitude of desired equals 0, skip out of here
        // (We could optimize this to check if x and y are 0 to avoid mag() square root
        if (v == 0)
        {
            return;
        }
//        System.out.printf("closing distance: %.2f\n", closingDist);

        // Scale with arbitrary damping within 5 cm
        if ( closingDist < 30 && arriving )
        {
            //float m = map(d,0,100,0,maxspeed);
            float m = (float) (0.0 + (maxspd - 0.0) * ((v - 0.0) / (30.0 - 0)));
            neededVelocity.setMag(m);
//            System.out.println("Arrive Now! -------------------------------------------");
        }
        else
        {
            neededVelocity.setMag(maxspd);
        }
        PVector neededAccel = PVector.sub(neededVelocity, velocity);
//        neededAccel.limit(maxforce);  // Limit to maximum steering force
        neededAccel.limit((float)(maxspd*0.2));  // Limit to maximum steering force
        acceleration.add(neededAccel);

        float neededAngularVelocity = hdg - heading;

        if ( Math.abs(neededAngularVelocity) < 90)
        {
            float velMag = (float) (0.0 + (5 - 0.0) * ((Math.abs(neededAngularVelocity) - 0.0) / (30.0 - 0)));
            neededAngularVelocity = velMag * Math.signum(neededAngularVelocity);
        }

        float neededAngularAccel = neededAngularVelocity - angularVelocity;
        angularAcceleration = neededAngularAccel;
        angularAcceleration = Range.clip(angularAcceleration, (float)-1, (float)1);
    }

}