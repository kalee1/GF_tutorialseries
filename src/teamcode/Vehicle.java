package teamcode;

// The Nature of Code
// Daniel Shiffman
// http://natureofcode.com

// Path Following

// Vehicle class

import processing.core.PVector;
import tbdServer.Range;

import java.util.ArrayList;
import java.util.List;

public class Vehicle
{
    // Vectors and parameters used to track the vehicle's motion and position.
    private PVector position;
    private PVector velocity;
    private PVector acceleration;
    private float maxAccel;
    private float maxSpeed;

    // Parameters that are used to track the vehicle's orientation.
    private float heading;
    private float angularVelocity;
    private float angularAcceleration;

    // Parameters used to track where along the path the vehicle is.
    private boolean arriving = false;
    private int currentPathSeg = 0;
    private float closingDist = (float)1000000.0;

    // Used for logging debug data.
    private CSVWriter logFile;
    private List<String> rowData;
    private boolean debug = false;

    // Constructor initialize all values
    public Vehicle(PVector l, float heading, float maxspd, float maxaccel)
    {
        position = l.copy();
        maxSpeed = maxspd;
        maxAccel = maxaccel;
        acceleration = new PVector(0, 0);
        velocity = new PVector(-maxSpeed, maxSpeed);

        if ( debug )
        {
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
            logFile.addRow(rowData);
            rowData.clear();
        }

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
    //
    public void follow(Path thePath)
    {

        // Based on which segment I'm on, I need to set a flag so that the seek method will know that is has to
        // arrive instead of just seeking.
        if (currentPathSeg == (thePath.points.size() - 2))
        {
            arriving = true;
        }

        // Set up vectors to use for determining which direction the vehicle should go
        PVector targetVector = null;
        PVector a = null;
        PVector b = null;
        PVector normalPoint = null;

        // Pull out 2 points to form the current segment of the path.
        // a is the beginning point of the path segment
        // b is the end point of the segment
        a = thePath.points.get(currentPathSeg);
        b = thePath.points.get(currentPathSeg + 1);

        // Calculate how far the vehicl is from the end point of the segment.
        // This will help me deal with properly transitioning to the next
        // segment.
        closingDist = PVector.dist(position, b);

        //
        // Step 1 - Preduct Future Position
        //
        // Predict position on the path a small amount ahead.
        // The predicted position is tweaked based on a few conditions below.
        //
        PVector predictedPosition = null;

        // If I am far enough away from b's "pointRadius", then
        // set my predicted position ahead 5 units in the direction of
        // my velocity.
        if ( closingDist > thePath.pointRadius )
        {
            PVector velocityCopy = velocity.copy();
            velocityCopy.setMag(5);

            // Add the scaled velocity vector to your current position to get the predicted future position
            predictedPosition = PVector.add(position, velocityCopy);
        }
        else if ( arriving )
        {
            // In this case, I am really close to point b, and I am "arriving"
            // which means that this is the last point on my path.  IN this case,
            // I want my target position to be the ending point, b.
            predictedPosition = b.copy();
        }
        else
        {
            // In this case, I am really close to point b, and I am somewhere
            // in the middle of the path (not at the end).  Since I am really close
            // to point b, I dont' want to extend my predction out too far,
            // so I go ahead and set the predicted position ot my current position.
            predictedPosition = position.copy();
        }

        if (debug)
        {
            rowData.add(String.valueOf(position.x));
            rowData.add(String.valueOf(position.y));
            rowData.add(String.valueOf(position.heading()));

            rowData.add(String.valueOf(predictedPosition.x));
            rowData.add(String.valueOf(predictedPosition.y));
            rowData.add(String.valueOf(closingDist));
        }

        //
        // Step 2 - Establish Normal Point on Path
        //
        // Now we must find the normal to the path from the predicted position.
        // This will give me the point on the path segment which is closest
        // to my current position.  This point will help me know where to set
        // my target next.
        //
        normalPoint = getNormalPoint(predictedPosition, a, b);

        // I need to do some checks on the normal point, because it's possible
        // that the normal calculation gives me a point I don't want to use.
        if (PVector.dist(normalPoint, b) > PVector.dist(a, b))
        {
            // In this case, my normal point is on the wrong side of the
            // beginning point a.  This normal point will cause the Vehicle
            // to steer away from b instead of towards it.  Thus, force the normal
            // point to be a copy of a.
            normalPoint = a.copy();
        }
        else if (PVector.dist(a, b) < PVector.dist(a, normalPoint))
        {
            // In this case, my normal point is on the wrong side of the
            // ending point b.  This normal point will cause the Vehicle
            // to steer beyond b instead of towards it.  Thus, force the normal
            // point to be a copy of b so as not to go beyond it.
            normalPoint = b.copy();
        }
        if (debug)
        {
            rowData.add(String.valueOf(normalPoint.x));
            rowData.add(String.valueOf(normalPoint.y));
        }

        //
        // Step 3 - Establish Target Point
        //
        // Now that I have a good normal point defined, use this normal point to
        // set my target down the path a small amount.
        //

        // Initially, set the target to the same as the normal
        targetVector = normalPoint.copy();

        // Look at the direction of the line segment so we can seek a little bit ahead of the normal
        PVector pathDir = PVector.sub(b, a);
        pathDir.setMag((float)(thePath.pointRadius));

        if (!arriving )
        {
            // If I am not on the last segment, go ahead and move the target
            // down the path a little bit.  This encourages a good aggressive
            // pursuit.
            targetVector.add(pathDir);

        }
        else if (closingDist > thePath.pointRadius)
        {
            // If I am on the final segment, go ahed and set the target down
            // the path as if I weren't on the final segment, but only if
            // I am not too close to the final point, b.
            targetVector.add(pathDir);
        }

        if (debug)
        {
            rowData.add(String.valueOf(a.x));
            rowData.add(String.valueOf(a.y));

            rowData.add(String.valueOf(b.x));
            rowData.add(String.valueOf(b.y));

            rowData.add(String.valueOf(targetVector.x));
            rowData.add(String.valueOf(targetVector.y));
            logFile.addRow(rowData);
            rowData.clear();
        }

        //
        // Step 4 - Seek Target Point
        //
        // Now that I have a good target defined, seek it out.  Also reference the
        // path segment-specific preferred heading and maxiumum speed. This allows
        // me to point the robot in a particular direction as well as speed up or
        // slow down while seeking/pursuing.
        //
        maxSpeed = thePath.maxspeeds.get(currentPathSeg);
        seek(targetVector, thePath.headings.get(currentPathSeg), thePath.maxspeeds.get(currentPathSeg));

        // If I'm pretty close to point b, go ahead and advance to the next segment
        // as long as I am not on the last segment.
        if ( !arriving && PVector.dist(b, normalPoint) < thePath.pointRadius )
        {
            currentPathSeg++;
        }

    }


    // A function to get the normal point from a point (p) to a line segment (a-b)
    // This function could be optimized to make fewer new Vector objects
    private PVector getNormalPoint(PVector p, PVector a, PVector b)
    {
        // Vector from a to p
        PVector ap = PVector.sub(p, a);
        // Vector from a to b
        PVector ab = PVector.sub(b, a);
        ab.normalize(); // Normalize the line
        // Project vector "diff" onto line by using the dot product
        ab.mult(ap.dot(ab));

        return PVector.add(a, ab);

    }

    // Method to update position
    public void update()
    {
        // Update velocity
        velocity.add(acceleration);
        // Limit speed
        velocity.limit(maxSpeed);
        position.add(velocity);
        // Reset accelertion to 0 each cycle
        acceleration.mult(0);

        angularVelocity += angularAcceleration;
        angularVelocity = Range.clip(angularVelocity, -10, 10);
        heading += angularVelocity;
    }

    // A method that calculates and applies steering towards a target
    private void seek(PVector target, float hdg, float maxspd)
    {
        // The difference between my target position and my current position is defined
        // as my needed velocity.
        PVector neededVelocity = PVector.sub(target, position);  // A vector pointing from the position to the target

        // Capture the magnitude of my needed velocity.
        float v = neededVelocity.mag();
        // If the magnitude of needed velocity equals 0, skip out of here because I don't need to move.
        if (v == 0)
        {
            return;
        }

        // If I am close to my end point of the path, I need to slow down.
        // Scale  my velocity in proportion to my distance from my final target.
        if ( closingDist < 30 && arriving )
        {
            float m = (float) (0.0 + (maxspd - 0.0) * ((v - 0.0) / (30.0 - 0)));
            neededVelocity.setMag(m);
//            System.out.println("Arrive Now! -------------------------------------------");
        }
        else
        {
            // If I am not close to my target, scale my needed velocity to maximum
            neededVelocity.setMag(maxspd);
        }

        // The difference between my current velocity and my needed velocity is
        // defined as my needed accleration.
        PVector neededAccel = PVector.sub(neededVelocity, velocity);
//        neededAccel.limit(maxforce);  // Limit to maximum steering force
        neededAccel.limit((float)(maxspd*0.2));  // Limit to maximum steering force to be proportional to the max speed.
        acceleration.add(neededAccel);

        // Peform similary velocity and accleration calculations for seeking the
        // desired heding for this segment.
        float neededAngularVelocity = hdg - heading;

        // If I am within 90 degrees of my target, scale down my needed angular velocity
        // proportional to how far away from my target I am.
        if ( Math.abs(neededAngularVelocity) < 60)
        {
            float velMag = (float) (0.0 + (6 - 0.0) * ((Math.abs(neededAngularVelocity) - 0.0) / (60.0 - 0)));
            neededAngularVelocity = velMag * Math.signum(neededAngularVelocity);
        }

        angularAcceleration = neededAngularVelocity - angularVelocity;
        angularAcceleration = Range.clip(angularAcceleration, (float)-maxAccel, (float)maxAccel);
    }

}
