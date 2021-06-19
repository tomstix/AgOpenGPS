using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;

namespace AgOpenGPS
{

    public class CRecordedPath
    {
        //constructor
        public CRecordedPath(FormGPS _f)
        {
            mf = _f;
        }

        //pointers to mainform controls
        private readonly FormGPS mf;

        //the recorded path from driving around
        public List<CRecPathPt> recList = new List<CRecPathPt>();

        //the dubins path to get there
        public List<CRecPathPt> shuttleDubinsList = new List<CRecPathPt>();

        //list of vec3 points of Dubins shortest path between 2 points - To be converted to RecPt
        public List<vec3> shortestDubinsList = new List<vec3>();

        public int C;

        //pure pursuit values
        public vec3 pivotAxlePosRP = new vec3(0, 0, 0);

        public bool isBtnFollowOn, isEndOfTheRecLine, isRecordOn;
        public bool isDrivingRecordedPath, isFollowingDubinsToPath, isFollowingRecPath, isFollowingDubinsHome;

        public bool StartDrivingRecordedPath()
        {
            //create the dubins path based on start and goal to start of recorded path
            C = 0;
            if (recList.Count < 5) return false;

            //get the dubins for approach to recorded path
            GetDubinsPath(new vec3(recList[0].easting, recList[0].northing, recList[0].heading));

            //has a valid dubins path been created?
            if (shuttleDubinsList.Count == 0) return false;

            //technically all good if we get here so set all the flags
            isFollowingDubinsHome = false;
            isFollowingRecPath = false;
            isFollowingDubinsToPath = true;
            isEndOfTheRecLine = false;
            isDrivingRecordedPath = true;
            return true;
        }

        public bool trig;
        public double north;
        public int pathCount = 0;

        public void UpdatePosition()
        {
            if (isFollowingDubinsToPath)
            {
                //set a speed of 10 kmh
                mf.sim.stepDistance = shuttleDubinsList[C].speed / 34.86;

                pivotAxlePosRP = mf.pivotAxlePos;

                List<vec3> tt = new List<vec3>(shuttleDubinsList);
                mf.gyd.CalculateSteerAngle(mf.pivotAxlePos, mf.steerAxlePos, ref tt, true, false);

                //check if close to recorded path
                int cnt = shuttleDubinsList.Count;
                pathCount = cnt - C;
                if (pathCount < 9)
                {
                    double distSqr = glm.DistanceSquared(pivotAxlePosRP.northing, pivotAxlePosRP.easting, recList[0].northing, recList[0].easting);
                    if (distSqr < 2)
                    {
                        isFollowingRecPath = true;
                        isFollowingDubinsToPath = false;
                        shuttleDubinsList.Clear();
                        shortestDubinsList.Clear();
                    }
                }
            }

            if (isFollowingRecPath)
            {
                pivotAxlePosRP = mf.pivotAxlePos;

                List<vec3> tt = new List<vec3>(recList);
                mf.gyd.CalculateSteerAngle(mf.pivotAxlePos, mf.steerAxlePos, ref tt, true, false);

                //if end of the line then stop
                if (!isEndOfTheRecLine)
                {
                    mf.sim.stepDistance = recList[C].speed / 34.86;
                    north = recList[C].northing;

                    pathCount = recList.Count - C;

                    //section control - only if different click the button
                    bool autoBtn = (mf.autoBtnState == FormGPS.btnStates.Auto);
                    trig = autoBtn;
                    if (autoBtn != recList[C].autoBtnState) mf.btnSectionOffAutoOn.PerformClick();
                }
                else
                {
                    StopDrivingRecordedPath();
                    return;

                    //create the dubins path based on start and goal to start trip home
                    //GetDubinsPath(homePos);
                    //shuttleListCount = shuttleDubinsList.Count;

                    ////its too small
                    //if (shuttleListCount < 3)
                    //{
                    //    StopDrivingRecordedPath();
                    //    return;
                    //}

                    ////set all the flags
                    //isFollowingDubinsHome = true;
                    //A = B = C = 0;
                    //isFollowingRecPath = false;
                    //isFollowingDubinsToPath = false;
                    //isEndOfTheRecLine = false;
                }
            }

            if (isFollowingDubinsHome)
            {
                int cnt = shuttleDubinsList.Count;
                pathCount = cnt - C;
                if (pathCount < 4)
                {
                    StopDrivingRecordedPath();
                    return;
                }

                mf.sim.stepDistance = shuttleDubinsList[C].speed / 35;
                pivotAxlePosRP = mf.pivotAxlePos;

                //List<vec3> tt = new List<vec3>(shuttleDubinsList);
                //mf.gyd.PurePursuitGuidance(mf.pivotAxlePos, ref tt, true);
            }
        }

        public void StopDrivingRecordedPath()
        {
            isFollowingDubinsHome = false;
            isFollowingRecPath = false;
            isFollowingDubinsToPath = false;
            shuttleDubinsList.Clear();
            shortestDubinsList.Clear();
            mf.sim.stepDistance = 0;
            isDrivingRecordedPath = false;
            mf.btnPathGoStop.Image = Properties.Resources.boundaryPlay;
            mf.btnPathRecordStop.Enabled = true;
            mf.btnPathDelete.Enabled = true;

        }

        private void GetDubinsPath(vec3 goal)
        {
            CDubins.turningRadius = mf.vehicle.minTurningRadius * 2.0;
            CDubins dubPath = new CDubins();

            // current psition
            pivotAxlePosRP = mf.pivotAxlePos;

            //bump it forward
            vec3 pt2 = new vec3(
                pivotAxlePosRP.easting + (Math.Sin(pivotAxlePosRP.heading) * 3),
                pivotAxlePosRP.northing + (Math.Cos(pivotAxlePosRP.heading) * 3),
                pivotAxlePosRP.heading);

            //get the dubins path vec3 point coordinates of turn
            shortestDubinsList.Clear();
            shuttleDubinsList.Clear();

            shortestDubinsList = dubPath.GenerateDubins(pt2, goal);

            //if Dubins returns 0 elements, there is an unavoidable blockage in the way.
            if (shortestDubinsList.Count > 0)
            {
                shortestDubinsList.Insert(0, mf.pivotAxlePos);

                //transfer point list to recPath class point style
                for (int i = 0; i < shortestDubinsList.Count; i++)
                {
                    CRecPathPt pt = new CRecPathPt(shortestDubinsList[i].easting, shortestDubinsList[i].northing, shortestDubinsList[i].heading, 9.0, false);
                    shuttleDubinsList.Add(pt);
                }
                return;
            }
        }

        public void DrawRecordedLine()
        {
            int ptCount = recList.Count;
            if (ptCount < 1) return;
            GL.LineWidth(1);
            GL.Color3(0.98f, 0.92f, 0.460f);
            GL.Begin(PrimitiveType.LineStrip);
            for (int h = 0; h < ptCount; h++) GL.Vertex3(recList[h].easting, recList[h].northing, 0);
            GL.End();


            if (shuttleDubinsList.Count > 1)
            {
                //GL.LineWidth(2);
                GL.PointSize(2);
                GL.Color3(0.298f, 0.96f, 0.2960f);
                GL.Begin(PrimitiveType.Points);
                for (int h = 0; h < shuttleDubinsList.Count; h++)
                    GL.Vertex3(shuttleDubinsList[h].easting, shuttleDubinsList[h].northing, 0);
                GL.End();
            }
        }
    }
}
//private void StanleyDubinsPath(int ptCount)
//{
//    //distanceFromCurrentLine = 9999;
//    //find the closest 2 points to current fix
//    double minDistA = 9999999999;
//    for (int t = 0; t < ptCount; t++)
//    {
//        double dist = ((pivotAxlePosRP.easting - shuttleDubinsList[t].easting) * (pivotAxlePosRP.easting - shuttleDubinsList[t].easting))
//                        + ((pivotAxlePosRP.northing - shuttleDubinsList[t].northing) * (pivotAxlePosRP.northing - shuttleDubinsList[t].northing));
//        if (dist < minDistA)
//        {
//            minDistA = dist;
//            A = t;
//        }
//    }

//    //save the closest point
//    C = A;
//    //next point is the next in list
//    B = A + 1;
//    if (B == ptCount) { A--; B--; }                //don't go past the end of the list - "end of the line" trigger

//    //get the distance from currently active AB line
//    //x2-x1
//    double dx = shuttleDubinsList[B].easting - shuttleDubinsList[A].easting;
//    //z2-z1
//    double dz = shuttleDubinsList[B].northing - shuttleDubinsList[A].northing;

//    if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return;

//    //abHeading = Math.Atan2(dz, dx);
//    abHeading = shuttleDubinsList[A].heading;

//    //how far from current AB Line is fix
//    distanceFromCurrentLinePivot = ((dz * pivotAxlePosRP.easting) - (dx * pivotAxlePosRP
//        .northing) + (shuttleDubinsList[B].easting
//                * shuttleDubinsList[A].northing) - (shuttleDubinsList[B].northing * shuttleDubinsList[A].easting))
//                    / Math.Sqrt((dz * dz) + (dx * dx));

//    //are we on the right side or not
//    isOnRightSideCurrentLine = distanceFromCurrentLinePivot > 0;

//    // calc point on ABLine closest to current position
//    double U = (((pivotAxlePosRP.easting - shuttleDubinsList[A].easting) * dx)
//                + ((pivotAxlePosRP.northing - shuttleDubinsList[A].northing) * dz))
//                / ((dx * dx) + (dz * dz));

//    rEastRP = shuttleDubinsList[A].easting + (U * dx);
//    rNorthRP = shuttleDubinsList[A].northing + (U * dz);

//    //the first part of stanley is to extract heading error
//    double abFixHeadingDelta = (pivotAxlePosRP.heading - abHeading);

//    //Fix the circular error - get it from -Pi/2 to Pi/2
//    if (abFixHeadingDelta > Math.PI) abFixHeadingDelta -= Math.PI;
//    else if (abFixHeadingDelta < Math.PI) abFixHeadingDelta += Math.PI;
//    if (abFixHeadingDelta > glm.PIBy2) abFixHeadingDelta -= Math.PI;
//    else if (abFixHeadingDelta < -glm.PIBy2) abFixHeadingDelta += Math.PI;

//    //normally set to 1, less then unity gives less heading error.
//    abFixHeadingDelta *= mf.vehicle.stanleyHeadingErrorGain;
//    if (abFixHeadingDelta > 0.74) abFixHeadingDelta = 0.74;
//    if (abFixHeadingDelta < -0.74) abFixHeadingDelta = -0.74;

//    //the non linear distance error part of stanley
//    steerAngleRP = Math.Atan((distanceFromCurrentLinePivot * mf.vehicle.stanleyDistanceErrorGain) / ((mf.pn.speed * 0.277777) + 1));

//    //clamp it to max 42 degrees
//    if (steerAngleRP > 0.74) steerAngleRP = 0.74;
//    if (steerAngleRP < -0.74) steerAngleRP = -0.74;

//    //add them up and clamp to max in vehicle settings
//    steerAngleRP = glm.toDegrees((steerAngleRP + abFixHeadingDelta) * -1.0);
//    if (steerAngleRP < -mf.vehicle.maxSteerAngle) steerAngleRP = -mf.vehicle.maxSteerAngle;
//    if (steerAngleRP > mf.vehicle.maxSteerAngle) steerAngleRP = mf.vehicle.maxSteerAngle;

//    //Convert to millimeters and round properly to above/below .5
//    distanceFromCurrentLinePivot = Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);

//    //every guidance method dumps into these that are used and sent everywhere, last one wins
//    mf.guidanceLineDistanceOff = mf.distanceDisplaySteer = (Int16)distanceFromCurrentLinePivot;
//    mf.guidanceLineSteerAngle = (Int16)(steerAngleRP * 100);
//}
//private void StanleyRecPath(int ptCount)
//{
//    //find the closest 2 points to current fix
//    double minDistA = 9999999999;

//    //set the search range close to current position
//    int top = currentPositonIndex + 5;
//    if (top > ptCount) top = ptCount;

//    double dist;
//    for (int t = currentPositonIndex; t < top; t++)
//    {
//        dist = ((pivotAxlePosRP.easting - recList[t].easting) * (pivotAxlePosRP.easting - recList[t].easting))
//                        + ((pivotAxlePosRP.northing - recList[t].northing) * (pivotAxlePosRP.northing - recList[t].northing));
//        if (dist < minDistA)
//        {
//            minDistA = dist;
//            A = t;
//        }
//    }

//    //Save the closest point
//    C = A;

//    //next point is the next in list
//    B = A + 1;
//    if (B == ptCount)
//    {
//        //don't go past the end of the list - "end of the line" trigger
//        A--;
//        B--;
//        isEndOfTheRecLine = true;
//    }

//    //save current position
//    currentPositonIndex = A;

//    //get the distance from currently active AB line
//    double dx = recList[B].easting - recList[A].easting;
//    double dz = recList[B].northing - recList[A].northing;

//    if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dz) < Double.Epsilon) return;

//    abHeading = Math.Atan2(dx, dz);
//    //abHeading = recList[A].heading;

//    //how far from current AB Line is fix
//    distanceFromCurrentLinePivot =
//        ((dz * pivotAxlePosRP.easting) - (dx * pivotAxlePosRP.northing) + (recList[B].easting
//                * recList[A].northing) - (recList[B].northing * recList[A].easting))
//                    / Math.Sqrt((dz * dz) + (dx * dx));

//    //are we on the right side or not
//    isOnRightSideCurrentLine = distanceFromCurrentLinePivot > 0;

//    // calc point on ABLine closest to current position
//    double U = (((pivotAxlePosRP.easting - recList[A].easting) * dx)
//                + ((pivotAxlePosRP.northing - recList[A].northing) * dz))
//                / ((dx * dx) + (dz * dz));

//    rEastRP = recList[A].easting + (U * dx);
//    rNorthRP = recList[A].northing + (U * dz);

//    //the first part of stanley is to extract heading error
//    double abFixHeadingDelta = (pivotAxlePosRP.heading - abHeading);

//    //Fix the circular error - get it from -Pi/2 to Pi/2
//    if (abFixHeadingDelta > Math.PI) abFixHeadingDelta -= Math.PI;
//    else if (abFixHeadingDelta < Math.PI) abFixHeadingDelta += Math.PI;
//    if (abFixHeadingDelta > glm.PIBy2) abFixHeadingDelta -= Math.PI;
//    else if (abFixHeadingDelta < -glm.PIBy2) abFixHeadingDelta += Math.PI;

//    //normally set to 1, less then unity gives less heading error.
//    abFixHeadingDelta *= mf.vehicle.stanleyHeadingErrorGain;
//    if (abFixHeadingDelta > 0.74) abFixHeadingDelta = 0.74;
//    if (abFixHeadingDelta < -0.74) abFixHeadingDelta = -0.74;

//    //the non linear distance error part of stanley
//    steerAngleRP = Math.Atan((distanceFromCurrentLinePivot * mf.vehicle.stanleyDistanceErrorGain) / ((mf.pn.speed * 0.277777) + 1));

//    //clamp it to max 42 degrees
//    if (steerAngleRP > 0.74) steerAngleRP = 0.74;
//    if (steerAngleRP < -0.74) steerAngleRP = -0.74;

//    //add them up and clamp to max in vehicle settings
//    steerAngleRP = glm.toDegrees((steerAngleRP + abFixHeadingDelta) * -1.0);
//    if (steerAngleRP < -mf.vehicle.maxSteerAngle) steerAngleRP = -mf.vehicle.maxSteerAngle;
//    if (steerAngleRP > mf.vehicle.maxSteerAngle) steerAngleRP = mf.vehicle.maxSteerAngle;

//    //Convert to millimeters and round properly to above/below .5
//    distanceFromCurrentLinePivot = Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);

//    //every guidance method dumps into these that are used and sent everywhere, last one wins
//    mf.guidanceLineDistanceOff = mf.distanceDisplaySteer = (Int16)distanceFromCurrentLinePivot;
//    mf.guidanceLineSteerAngle = (Int16)(steerAngleRP * 100);
//}