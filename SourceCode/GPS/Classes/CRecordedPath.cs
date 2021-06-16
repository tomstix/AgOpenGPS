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
                mf.gyd.PurePursuitGuidance(mf.pivotAxlePos, ref tt, true);

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
                mf.gyd.PurePursuitGuidance(mf.pivotAxlePos, ref tt, true);

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