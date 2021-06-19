using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CYouTurn
    {
        //copy of the mainform address
        private readonly FormGPS mf;

        /// <summary>/// triggered right after youTurnTriggerPoint is set /// </summary>
        public bool isYouTurnTriggered;

        /// <summary>  /// turning right or left?/// </summary>
        public bool isYouTurnRight, isLastToggle;

        /// <summary> /// Is the youturn button enabled? /// </summary>
        public bool isYouTurnBtnOn;

        public double boundaryAngleOffPerpendicular;
        public double distanceTurnBeforeLine = 0, tangencyAngle;

        public int rowSkipsWidth = 1, uTurnSmoothing = 10;

        public bool alternateSkips = false, previousBigSkip = true;
        public int rowSkipsWidth2 = 3, turnSkips = 2;

        /// <summary>  /// distance from headland as offset where to start turn shape /// </summary>
        public int youTurnStartOffset;

        //guidance values
        public double uturnDistanceFromBoundary;

        public bool isTurnCreationTooClose = false, isTurnCreationNotCrossingError = false;

        //list of points for scaled and rotated YouTurn line, used for pattern, dubins, abcurve, abline
        public List<vec3> ytList = new List<vec3>();

        //is UTurn pattern in or out of bounds
        public bool isOutOfBounds = false;

        //sequence of operations of finding the next turn 0 to 3
        public int youTurnPhase = -1, curListCount;

        vec6 Crossing = new vec6(0, 0, double.MaxValue, -1, -1, -1);
        public vec3 crossingCurvePoint = new vec3(0.0,0.0,0.0);
        private double crossingHeading = 0;
        private int crossingIndex = 0;

        public CYouTurn(FormGPS _f)
        {
            mf = _f;

            uturnDistanceFromBoundary = Properties.Vehicle.Default.set_youTurnDistanceFromBoundary;

            //how far before or after boundary line should turn happen
            youTurnStartOffset = Properties.Vehicle.Default.set_youTurnExtensionLength;

            rowSkipsWidth = Properties.Vehicle.Default.set_youSkipWidth;
            Set_Alternate_skips();

            ytList.Capacity = 128;
        }

        public bool GetLineIntersection(vec2 PointAA, vec2 PointAB, vec2 PointBA, vec2 PointBB, out vec2 Crossing, out double TimeA, bool Limit = false)
        {
            TimeA = -1;
            Crossing = new vec2(0.0,0.0);
            double denominator = (PointAB.northing - PointAA.northing) * (PointBB.easting - PointBA.easting) - (PointBB.northing - PointBA.northing) * (PointAB.easting - PointAA.easting);

            if (denominator != 0.0)
            {
                TimeA = ((PointBB.northing - PointBA.northing) * (PointAA.easting - PointBA.easting) - (PointAA.northing - PointBA.northing) * (PointBB.easting - PointBA.easting)) / denominator;

                if (Limit || (TimeA > 0.0 && TimeA < 1.0))
                {
                    double TimeB = ((PointAB.northing - PointAA.northing) * (PointAA.easting - PointBA.easting) - (PointAA.northing - PointBA.northing) * (PointAB.easting - PointAA.easting)) / denominator;
                    if (Limit || (TimeB > 0.0 && TimeB < 1.0))
                    {
                        Crossing = PointAA + (PointAB - PointAA) * TimeA;
                        return true;
                    }
                    else return false;
                }
                else return false;
            }
            else return false;
        }

        //Finds the point where an AB Curve crosses the turn line
        public bool FindTurnPoint(ref List<vec3> Points, bool isSameWay)
        {
            //find closet AB Curve point that will cross and go out of bounds
            curListCount = Points.Count;

            int Count = isSameWay ? 1 : -1;
            Crossing = new vec6(0,0,double.MaxValue,-1,-1,-1);

            vec3 Start = new vec3(mf.gyd.rEast, mf.gyd.rNorth, 0);

            for (int i = mf.gyd.currentLocationIndex; i >= 0 && i < Points.Count; i += Count)
            {
                for (int j = 0; j < mf.bnd.bndArr.Count; j++)
                {
                    if (!mf.bnd.bndArr[j].isSet) continue;
                    if (mf.bnd.bndArr[j].isDriveThru) continue;
                    if (mf.bnd.bndArr[j].isDriveAround) continue;

                    if (mf.turn.turnArr[j].turnLine.Count > 1)
                    {
                        int k = mf.turn.turnArr[j].turnLine.Count - 1;
                        for (int l = 0; l < mf.turn.turnArr[j].turnLine.Count; l++)
                        {
                            if (GetLineIntersection(Start, Points[i], mf.turn.turnArr[j].turnLine[l], mf.turn.turnArr[j].turnLine[k], out vec2 _Crossing, out double Time))
                            {
                                if (Time < Crossing.time)
                                    Crossing = new vec6(_Crossing.easting, _Crossing.northing, Time, j, i, l);
                            }
                            k = l;
                        }
                    }
                }

                if (Crossing.boundaryIndex >= 0)
                {
                    mf.distancePivotToTurnLine = glm.Distance(mf.pivotAxlePos, Crossing.easting, Crossing.northing);

                    break;
                }
                Start = Points[i];
            }

            if (Crossing.boundaryIndex == -1)
            {
                isTurnCreationNotCrossingError = true;
                return false;
            }

            crossingIndex = Crossing.crosssingIdx;

            crossingHeading = mf.turn.turnArr[Crossing.boundaryIndex].turnLine[Crossing.turnLineIdx].heading;

            crossingCurvePoint.easting = Points[crossingIndex].easting;
            crossingCurvePoint.northing = Points[crossingIndex].northing;
            crossingCurvePoint.heading = Points[crossingIndex].heading;

            return true;
        }

        public void AddSequenceLines(double head)
        {
            for (int a = 0; a < youTurnStartOffset * 2; a++)
            {
                ytList.Insert(0, new vec3(
                    ytList[0].easting + (Math.Sin(head) * 0.2),
                    ytList[0].northing + (Math.Cos(head) * 0.2),
                    ytList[0].heading));
            }

            int count = ytList.Count;

            for (int i = 1; i <= youTurnStartOffset * 2; i++)
            {
                ytList.Add(new vec3(
                    ytList[count - 1].easting + (Math.Sin(head) * i * 0.2),
                    ytList[count - 1].northing + (Math.Cos(head) * i * 0.2),
                    head));
            }

            double distancePivotToTurnLine;
            count = ytList.Count;
            for (int i = 0; i < count; i += 2)
            {
                distancePivotToTurnLine = glm.Distance(ytList[i], mf.pivotAxlePos);
                if (distancePivotToTurnLine > 3)
                {
                    isTurnCreationTooClose = false;
                }
                else
                {
                    isTurnCreationTooClose = true;
                    //set the flag to Critical stop machine
                    if (isTurnCreationTooClose) mf.mc.isOutOfBounds = true;
                    break;
                }
            }
        }

        ////list of points of collision path avoidance
        //public List<vec3> mazeList = new List<vec3>();

        //public bool BuildDriveAround()
        //{
        //    double headAB = mf.ABLine.abHeading;
        //    if (!mf.ABLine.isABSameAsVehicleHeading) headAB += Math.PI;

        //    double cosHead = Math.Cos(headAB);
        //    double sinHead = Math.Sin(headAB);

        //    vec3 start = new vec3();
        //    vec3 stop = new vec3();
        //    vec3 pt2 = new vec3();

        //    //grab the pure pursuit point right on ABLine
        //    vec3 onPurePoint = new vec3(mf.ABLine.rEastAB, mf.ABLine.rNorthAB, 0);

        //    //how far are we from any geoFence
        //    mf.gf.FindPointsDriveAround(onPurePoint, headAB, ref start, ref stop);

        //    //not an inside border
        //    if (start.easting == 88888) return false;

        //    //get the dubins path vec3 point coordinates of path
        //    ytList?.Clear();

        //    //find a path from start to goal - diagnostic, but also used later
        //    mazeList = mf.mazeGrid.SearchForPath(start, stop);

        //    //you can't get anywhere!
        //    if (mazeList == null) return false;

        //    //not really changing direction so need to fake a turn twice.
        //    mf.SwapDirection();

        //    //list of vec3 points of Dubins shortest path between 2 points - To be converted to RecPt
        //    List<vec3> shortestDubinsList = new List<vec3>();

        //    //Dubins at the start and stop of mazePath
        //    CDubins.turningRadius = mf.vehicle.minTurningRadius * 1.0;
        //    CDubins dubPath = new CDubins();

        //    //start is navigateable - maybe
        //    int cnt = mazeList.Count;
        //    int cut = 8;
        //    if (cnt < 18) cut = 3;

        //    if (cnt > 0)
        //    {
        //        pt2.easting = start.easting - (sinHead * mf.vehicle.minTurningRadius * 1.5);
        //        pt2.northing = start.northing - (cosHead * mf.vehicle.minTurningRadius * 1.5);
        //        pt2.heading = headAB;

        //        shortestDubinsList = dubPath.GenerateDubins(pt2, mazeList[cut - 1], mf.gf);
        //        for (int i = 1; i < shortestDubinsList.Count; i++)
        //        {
        //            vec3 pt = new vec3(shortestDubinsList[i].easting, shortestDubinsList[i].northing, shortestDubinsList[i].heading);
        //            ytList.Add(pt);
        //        }

        //        for (int i = cut; i < mazeList.Count - cut; i++)
        //        {
        //            vec3 pt = new vec3(mazeList[i].easting, mazeList[i].northing, mazeList[i].heading);
        //            ytList.Add(pt);
        //        }

        //        pt2.easting = stop.easting + (sinHead * mf.vehicle.minTurningRadius * 1.5);
        //        pt2.northing = stop.northing + (cosHead * mf.vehicle.minTurningRadius * 1.5);
        //        pt2.heading = headAB;

        //        shortestDubinsList = dubPath.GenerateDubins(mazeList[cnt - cut], pt2, mf.gf);

        //        for (int i = 1; i < shortestDubinsList.Count; i++)
        //        {
        //            vec3 pt = new vec3(shortestDubinsList[i].easting, shortestDubinsList[i].northing, shortestDubinsList[i].heading);
        //            ytList.Add(pt);
        //        }
        //    }

        //    if (ytList.Count > 10) youTurnPhase = 3;

        //    vec3 pt3 = new vec3();

        //    for (int a = 0; a < youTurnStartOffset; a++)
        //    {
        //        pt3.easting = ytList[0].easting - sinHead;
        //        pt3.northing = ytList[0].northing - cosHead;
        //        pt3.heading = headAB;
        //        ytList.Insert(0, pt3);
        //    }

        //    int count = ytList.Count;

        //    for (int i = 1; i <= youTurnStartOffset; i++)
        //    {
        //        pt3.easting = ytList[count - 1].easting + (sinHead * i);
        //        pt3.northing = ytList[count - 1].northing + (cosHead * i);
        //        pt3.heading = headAB;
        //        ytList.Add(pt3);
        //    }

        //    return true;
        //}

        public bool BuildABLineDubinsYouTurn(bool isTurnRight)
        {
            double headAB = mf.ABLine.abHeading;
            if (!mf.ABLine.isHeadingSameWay) headAB += Math.PI;

            if (youTurnPhase == 0)
            {
                if (!FindTurnPoint(ref mf.ABLine.curlist, mf.ABLine.isHeadingSameWay))
                {
                    //Full emergency stop code goes here, it thinks its auto turn, but its not!
                    mf.distancePivotToTurnLine = -3333;
                    ResetCreatedYouTurn();
                    return false;
                }

                //delta between AB heading and boundary closest point heading
                boundaryAngleOffPerpendicular = Math.PI - Math.Abs(Math.Abs(crossingCurvePoint.heading - headAB) - Math.PI);
                boundaryAngleOffPerpendicular -= glm.PIBy2;
                boundaryAngleOffPerpendicular *= -1;
                if (boundaryAngleOffPerpendicular > 1.25) boundaryAngleOffPerpendicular = 1.25;
                if (boundaryAngleOffPerpendicular < -1.25) boundaryAngleOffPerpendicular = -1.25;

                //for calculating innner circles of turn
                tangencyAngle = (glm.PIBy2 - Math.Abs(boundaryAngleOffPerpendicular)) * 0.5;

                //baseline away from boundary to start calculations
                double toolTurnWidth = mf.tool.toolWidth * rowSkipsWidth;

                //distance from TurnLine for trigger added in youturn form, include the 3 m bump forward
                distanceTurnBeforeLine = 0;

                if (mf.vehicle.minTurningRadius * 2 < toolTurnWidth)
                {
                    if (boundaryAngleOffPerpendicular < 0)
                    {
                        //which is actually left
                        if (isYouTurnRight)
                            distanceTurnBeforeLine += (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle));//short
                        else
                            distanceTurnBeforeLine += (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                    }
                    else
                    {
                        //which is actually left
                        if (isYouTurnRight)
                            distanceTurnBeforeLine += (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                        else
                            distanceTurnBeforeLine += (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                    }
                }
                else //turn Radius is wider then equipment width so ohmega turn
                {
                    distanceTurnBeforeLine += (2 * mf.vehicle.minTurningRadius);
                }

                //used for distance calc for other part of turn

                CDubins dubYouTurnPath = new CDubins();
                CDubins.turningRadius = mf.vehicle.minTurningRadius;

                double head = mf.ABLine.abHeading;

                //grab the vehicle widths and offsets
                double turnOffset = (mf.tool.toolWidth - mf.tool.toolOverlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.toolOffset * 2.0 : mf.tool.toolOffset * 2.0);

                double turnRadius = turnOffset / Math.Cos(boundaryAngleOffPerpendicular);
                if (!mf.ABLine.isHeadingSameWay) head += Math.PI;

                double turnDiagDistance = mf.distancePivotToTurnLine;

                //move the start forward 2 meters
                vec3 start = new vec3(mf.gyd.rEast + Math.Sin(head) * turnDiagDistance, mf.gyd.rNorth + Math.Cos(head) * turnDiagDistance, head);
                //move the cross line calc to not include first turn
                vec3 goal = new vec3(
                    start.easting + (Math.Sin(head) * distanceTurnBeforeLine),
                    start.northing + (Math.Cos(head) * distanceTurnBeforeLine), 0);

                //headland angle relative to vehicle heading to head along the boundary left or right
                double bndAngle = head - boundaryAngleOffPerpendicular + glm.PIBy2;

                //now we go the other way to turn round
                head -= Math.PI;
                if (head < 0) head += glm.twoPI;

                if ((mf.vehicle.minTurningRadius * 2.0) < turnOffset)
                {
                    //are we right of boundary
                    if (boundaryAngleOffPerpendicular > 0)
                    {
                        if (!isYouTurnRight) //which is actually right now
                        {
                            goal.easting += (Math.Sin(bndAngle) * turnRadius);
                            goal.northing += (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                        else //going left
                        {
                            goal.easting -= (Math.Sin(bndAngle) * turnRadius);
                            goal.northing -= (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                    }
                    else // going left of boundary
                    {
                        if (!isYouTurnRight) //pointing to right
                        {
                            goal.easting += (Math.Sin(bndAngle) * turnRadius);
                            goal.northing += (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                        else
                        {
                            goal.easting -= (Math.Sin(bndAngle) * turnRadius);
                            goal.northing -= (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                    }
                }
                else
                {
                    if (!isTurnRight)
                    {
                        goal.easting = start.easting - (Math.Cos(-head) * turnOffset);
                        goal.northing = start.northing - (Math.Sin(-head) * turnOffset);
                    }
                    else
                    {
                        goal.easting = start.easting + (Math.Cos(-head) * turnOffset);
                        goal.northing = start.northing + (Math.Sin(-head) * turnOffset);
                    }
                    goal.easting += (Math.Sin(head) * 1);
                    goal.northing += (Math.Cos(head) * 1);
                    goal.heading = head;

                }

                goal.heading = head;

                //generate the turn points
                ytList = dubYouTurnPath.GenerateDubins(start, goal);
                AddSequenceLines(head);

                if (ytList.Count == 0) return false;
                else youTurnPhase = 1;
            }

            if (youTurnPhase == 3) return true;

            // Phase 0 - back up the turn till it is out of bounds.
            // Phase 1 - move it forward till out of bounds.
            // Phase 2 - move forward couple meters away from turn line.
            // Phase 3 - ytList is made, waiting to get close enough to it

            isOutOfBounds = false;
            switch (youTurnPhase)
            {
                case 1:
                    //the temp array
                    mf.distancePivotToTurnLine = glm.Distance(ytList[0], mf.pivotAxlePos);
                    double cosHead = Math.Cos(headAB);
                    double sinHead = Math.Sin(headAB);

                    int cnt = ytList.Count;

                    for (int i = 0; i < cnt; i++)
                    {
                        ytList[i].easting -= (sinHead);
                        ytList[i].northing -= (cosHead);
                    }

                    for (int j = 0; j < cnt; j += 2)
                    {
                        if (!mf.turn.turnArr[0].IsPointInTurnWorkArea(ytList[j])) isOutOfBounds = true;
                        if (isOutOfBounds) break;

                        for (int i = 1; i < mf.bnd.bndArr.Count; i++)
                        {
                            //make sure not inside a non drivethru boundary
                            if (!mf.bnd.bndArr[i].isSet) continue;
                            if (mf.bnd.bndArr[i].isDriveThru) continue;
                            if (mf.bnd.bndArr[i].isDriveAround) continue;
                            if (mf.turn.turnArr[i].IsPointInTurnWorkArea(ytList[j]))
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }
                        if (isOutOfBounds) break;
                    }

                    if (!isOutOfBounds)
                    {
                        youTurnPhase = 2;
                    }
                    else
                    {
                        //turn keeps approaching vehicle and running out of space - end of field?
                        if (isOutOfBounds && mf.distancePivotToTurnLine > 3)
                        {
                            isTurnCreationTooClose = false;
                        }
                        else
                        {
                            isTurnCreationTooClose = true;

                            //set the flag to Critical stop machine
                            if (isTurnCreationTooClose) mf.mc.isOutOfBounds = true;
                        }
                    }
                    break;

                //move again out of bounds
                case 2:
                    //the temp array
                    mf.distancePivotToTurnLine = glm.Distance(ytList[0], mf.pivotAxlePos);
                    cosHead = Math.Cos(headAB);
                    sinHead = Math.Sin(headAB);

                    cnt = ytList.Count;

                    for (int i = 0; i < cnt; i++)
                    {
                        ytList[i].easting += (sinHead * 0.05);
                        ytList[i].northing += (cosHead * 0.05);
                    }

                    for (int j = 0; j < cnt; j += 2)
                    {
                        if (!mf.turn.turnArr[0].IsPointInTurnWorkArea(ytList[j])) isOutOfBounds = true;
                        if (isOutOfBounds)
                            break;

                        for (int i = 1; i < mf.bnd.bndArr.Count; i++)
                        {
                            //make sure not inside a non drivethru boundary
                            if (!mf.bnd.bndArr[i].isSet) continue;
                            if (mf.bnd.bndArr[i].isDriveThru) continue;
                            if (mf.bnd.bndArr[i].isDriveAround) continue;
                            if (!mf.turn.turnArr[i].IsPointInTurnWorkArea(ytList[j]))
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }
                        if (isOutOfBounds)
                            break;
                    }

                    if (isOutOfBounds)
                    {
                        isOutOfBounds = false;
                        youTurnPhase = 3;
                    }
                    else
                    {
                        //turn keeps approaching vehicle and running out of space - end of field?
                        if (!isOutOfBounds && mf.distancePivotToTurnLine > 3)
                        {
                            isTurnCreationTooClose = false;
                        }
                        else
                        {
                            isTurnCreationTooClose = true;

                            //set the flag to Critical stop machine
                            if (isTurnCreationTooClose) mf.mc.isOutOfBounds = true;
                        }
                    }
                    break;
            }
            return true;
        }

        public bool BuildCurveDubinsYouTurn(bool isTurnRight, vec3 pivotPos)
        {
            if (youTurnPhase > 0)
            {
                double head = crossingCurvePoint.heading;
                if (!mf.curve.isHeadingSameWay) head += Math.PI;

                //delta between AB heading and boundary closest point heading
                boundaryAngleOffPerpendicular = Math.PI - Math.Abs(Math.Abs(crossingHeading - head) - Math.PI);
                boundaryAngleOffPerpendicular -= glm.PIBy2;
                boundaryAngleOffPerpendicular *= -1;
                if (boundaryAngleOffPerpendicular > 1.25) boundaryAngleOffPerpendicular = 1.25;
                if (boundaryAngleOffPerpendicular < -1.25) boundaryAngleOffPerpendicular = -1.25;

                //for calculating innner circles of turn
                tangencyAngle = (glm.PIBy2 - Math.Abs(boundaryAngleOffPerpendicular)) * 0.5;

                //distance from crossPoint to turn line
                if (mf.vehicle.minTurningRadius * 2 < (mf.tool.toolWidth * rowSkipsWidth))
                {
                    if (boundaryAngleOffPerpendicular < 0)
                    {
                        //which is actually left
                        if (isYouTurnRight)
                            distanceTurnBeforeLine = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle));//short
                        else
                            distanceTurnBeforeLine = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                    }
                    else
                    {
                        //which is actually left
                        if (isYouTurnRight)
                            distanceTurnBeforeLine = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                        else
                            distanceTurnBeforeLine = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                    }
                }

                //turn Radius is wider then equipment width so ohmega turn
                else
                {
                    distanceTurnBeforeLine = (2 * mf.vehicle.minTurningRadius);
                }

                CDubins dubYouTurnPath = new CDubins();
                CDubins.turningRadius = mf.vehicle.minTurningRadius;

                //grab the vehicle widths and offsets
                double turnOffset = (mf.tool.toolWidth - mf.tool.toolOverlap) * rowSkipsWidth + (isYouTurnRight ? -mf.tool.toolOffset * 2.0 : mf.tool.toolOffset * 2.0);

                //diagonally across
                double turnRadius = turnOffset / Math.Cos(boundaryAngleOffPerpendicular);

                //start point of Dubins
                vec3 start = new vec3(crossingCurvePoint.easting, crossingCurvePoint.northing, head);

                //move the cross line calc to not include first turn
                vec3 goal = new vec3(
                    crossingCurvePoint.easting + (Math.Sin(head) * distanceTurnBeforeLine),
                    crossingCurvePoint.northing + (Math.Cos(head) * distanceTurnBeforeLine), 0);

                //headland angle relative to vehicle heading to head along the boundary left or right
                double bndAngle = head - boundaryAngleOffPerpendicular + glm.PIBy2;

                //now we go the other way to turn round
                head -= Math.PI;
                if (head < 0) head += glm.twoPI;

                if ((mf.vehicle.minTurningRadius * 2.0) < turnOffset)
                {
                    //are we right of boundary
                    if (boundaryAngleOffPerpendicular > 0)
                    {
                        if (!isYouTurnRight) //which is actually right now
                        {
                            goal.easting += (Math.Sin(bndAngle) * turnRadius);
                            goal.northing += (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                        else //going left
                        {
                            goal.easting -= (Math.Sin(bndAngle) * turnRadius);
                            goal.northing -= (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                    }
                    else // going left of boundary
                    {
                        if (!isYouTurnRight) //pointing to right
                        {
                            goal.easting += (Math.Sin(bndAngle) * turnRadius);
                            goal.northing += (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius * Math.Tan(tangencyAngle)); //short
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                        else
                        {
                            goal.easting -= (Math.Sin(bndAngle) * turnRadius);
                            goal.northing -= (Math.Cos(bndAngle) * turnRadius);

                            double dis = (mf.vehicle.minTurningRadius / Math.Tan(tangencyAngle)); //long
                            goal.easting += (Math.Sin(head) * dis);
                            goal.northing += (Math.Cos(head) * dis);
                        }
                    }
                }
                else
                {
                    if (!isTurnRight)
                    {
                        goal.easting = crossingCurvePoint.easting - (Math.Cos(-head) * turnOffset);
                        goal.northing = crossingCurvePoint.northing - (Math.Sin(-head) * turnOffset);
                    }
                    else
                    {
                        goal.easting = crossingCurvePoint.easting + (Math.Cos(-head) * turnOffset);
                        goal.northing = crossingCurvePoint.northing + (Math.Sin(-head) * turnOffset);
                    }
                }

                goal.heading = head;

                //goal.easting += (Math.Sin(head) * 0.5);
                //goal.northing += (Math.Cos(head) * 0.5);
                //goal.heading = head;

                //generate the turn points
                ytList = dubYouTurnPath.GenerateDubins(start, goal);
                int count = ytList.Count;
                if (count == 0) return false;

                //these are the lead in lead out lines that add to the turn
                AddSequenceLines(head);
            }

            switch (youTurnPhase)
            {
                case 0: //find the crossing points
                    if (FindTurnPoint(ref mf.curve.curList, mf.curve.isHeadingSameWay)) youTurnPhase = 1;
                    ytList?.Clear();
                    break;

                case 1:
                    //now check to make sure we are not in an inner turn boundary - drive thru is ok
                    int count = ytList.Count;
                    if (count == 0) return false;

                    //Are we out of bounds?
                    isOutOfBounds = false;
                    for (int j = 0; j < count; j += 2)
                    {
                        if (!mf.turn.turnArr[0].IsPointInTurnWorkArea(ytList[j]))
                        {
                            isOutOfBounds = true;
                            break;
                        }

                        for (int i = 1; i < mf.bnd.bndArr.Count; i++)
                        {
                            //make sure not inside a non drivethru boundary
                            if (!mf.bnd.bndArr[i].isSet) continue;
                            if (mf.bnd.bndArr[i].isDriveThru) continue;
                            if (mf.bnd.bndArr[i].isDriveAround) continue;
                            if (mf.turn.turnArr[i].IsPointInTurnWorkArea(ytList[j]))
                            {
                                isOutOfBounds = true;
                                break;
                            }
                        }
                        if (isOutOfBounds) break;
                    }

                    //first check if not out of bounds, add a bit more to clear turn line, set to phase 2
                    if (!isOutOfBounds)
                    {
                        youTurnPhase = 2;
                        //if (mf.curve.isABSameAsVehicleHeading)
                        //{
                        //   crossingIndex -= 2;
                        //    if (crossingIndex < 0)crossingIndex = 0;
                        //}
                        //else
                        //{
                        //    crossingIndex += 2;
                        //    if (crossingIndex >= curListCount)
                        //        crossingIndex = curListCount - 1;
                        //}
                        //crossingCurvePoint.easting = mf.curve.curList[crossingIndex].easting;
                        //crossingCurvePoint.northing = mf.curve.curList[crossingIndex].northing;
                        //crossingCurvePoint.heading = mf.curve.curList[crossingIndex].heading;
                        return true;
                    }

                    //keep moving infield till pattern is all inside
                    if (mf.curve.isHeadingSameWay)
                    {
                        crossingIndex--;
                        if (crossingIndex < 0) crossingIndex = 0;
                    }
                    else
                    {
                        crossingIndex++;
                        if (crossingIndex >= curListCount)
                            crossingIndex = curListCount - 1;
                    }
                    crossingCurvePoint.easting = mf.curve.curList[crossingIndex].easting;
                    crossingCurvePoint.northing = mf.curve.curList[crossingIndex].northing;
                    crossingCurvePoint.heading = mf.curve.curList[crossingIndex].heading;

                    double tooClose = glm.Distance(ytList[0], pivotPos);
                    isTurnCreationTooClose = tooClose < 3;

                    //set the flag to Critical stop machine
                    if (isTurnCreationTooClose) mf.mc.isOutOfBounds = true;
                    break;

                case 2:
                    youTurnPhase = 3;
                    break;
            }
            return true;
        }

        public void SmoothYouTurn(int smPts)
        {
            //count the reference list of original curve
            int cnt = ytList.Count;

            //the temp array
            vec3[] arr = new vec3[cnt];

            //read the points before and after the setpoint
            for (int s = 0; s < smPts / 2; s++)
            {
                arr[s] = new vec3(ytList[s]);
            }

            //average them - center weighted average
            for (int i = smPts / 2; i < cnt - (smPts / 2); i++)
            {
                double easting = 0;
                double northing = 0;
                for (int j = -smPts / 2; j < smPts / 2; j++)
                {
                    easting += ytList[j + i].easting;
                    northing += ytList[j + i].northing;
                }
                easting /= smPts;
                northing /= smPts;
                arr[i] = new vec3(easting, northing, ytList[i].heading);
            }

            for (int s = cnt - (smPts / 2); s < cnt; s++)
            {
                arr[s] = new vec3(ytList[s]);
            }

            ytList?.Clear();

            //calculate new headings on smoothed line
            for (int i = 1; i < cnt - 1; i++)
            {
                arr[i].heading = Math.Atan2(arr[i + 1].easting - arr[i].easting, arr[i + 1].northing - arr[i].northing);
                if (arr[i].heading < 0) arr[i].heading += glm.twoPI;
                ytList.Add(arr[i]);
            }
        }

        //called to initiate turn
        public void YouTurnTrigger()
        {
            //trigger pulled
            isYouTurnTriggered = true;

            if (alternateSkips && rowSkipsWidth2 > 1)
            {
                if (--turnSkips == 0)
                {
                    isYouTurnRight = !isYouTurnRight;
                    turnSkips = rowSkipsWidth2 * 2 - 1;
                }
                else if (previousBigSkip = !previousBigSkip)
                    rowSkipsWidth = rowSkipsWidth2 - 1;
                else
                    rowSkipsWidth = rowSkipsWidth2;
            }
            else isYouTurnRight = !isYouTurnRight;

            mf.guidanceLookPos.easting = ytList[ytList.Count - 1].easting;
            mf.guidanceLookPos.northing = ytList[ytList.Count - 1].northing;

            mf.gyd.isLateralTriggered = true;
            if (mf.ABLine.isABLineSet)
                mf.ABLine.isABValid = false;
            else
                mf.curve.isCurveValid = false;
        }

        public void Set_Alternate_skips()
        {
            rowSkipsWidth2 = rowSkipsWidth;
            turnSkips = rowSkipsWidth2 * 2 - 1;
            previousBigSkip = false;
        }

        //something went seriously wrong so reset everything
        public void ResetYouTurn()
        {
            isYouTurnTriggered = false;
            mf.isBoundAlarming = false;
            //fix you turn
            ResetCreatedYouTurn();
            isTurnCreationTooClose = false;
            isTurnCreationNotCrossingError = false;
        }

        public void ResetCreatedYouTurn()
        {
            youTurnPhase = -1;
            ytList?.Clear();
        }

        public void BuildManualYouLateral(bool isTurnRight)
        {
            double head = mf.gyd.UturnHeading;

            mf.gyd.isLateralTriggered = true;

            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.toolWidth - mf.tool.toolOverlap); //remove rowSkips

            //if its straight across it makes 2 loops instead so goal is a little lower then start
            if (!(mf.ABLine.isBtnABLineOn ? mf.ABLine.isHeadingSameWay : mf.curve.isHeadingSameWay)) head += Math.PI;

            //move the start forward 2 meters, this point is critical to formation of uturn
            if (isTurnRight)
            {
                mf.guidanceLookPos.easting = mf.gyd.rEast + Math.Sin(head) * 2 + (Math.Cos(-head) * turnOffset);
                mf.guidanceLookPos.northing = mf.gyd.rNorth + Math.Cos(head) * 2 + (Math.Sin(-head) * turnOffset);
            }
            else
            {
                mf.guidanceLookPos.easting = mf.gyd.rEast + Math.Sin(head) * 2 - (Math.Cos(-head) * turnOffset);
                mf.guidanceLookPos.northing = mf.gyd.rNorth + Math.Cos(head) * 2 - (Math.Sin(-head) * turnOffset);
            }

            mf.ABLine.isABValid = false;
            mf.curve.isCurveValid = false;
        }

        //build the points and path of youturn to be scaled and transformed
        public void BuildManualYouTurn(bool isTurnRight)
        {
            isYouTurnTriggered = true;

            double head = mf.gyd.UturnHeading;

            mf.gyd.isLateralTriggered = true;

            //grab the vehicle widths and offsets
            double turnOffset = (mf.tool.toolWidth - mf.tool.toolOverlap) * rowSkipsWidth + (isTurnRight ? mf.tool.toolOffset * 2.0 : -mf.tool.toolOffset * 2.0);

            CDubins dubYouTurnPath = new CDubins();
            CDubins.turningRadius = mf.vehicle.minTurningRadius;

            //if its straight across it makes 2 loops instead so goal is a little lower then start
            if (!(mf.ABLine.isBtnABLineOn ? mf.ABLine.isHeadingSameWay : mf.curve.isHeadingSameWay)) head += 3.14;
            else head -= 0.01;

            //move the start forward 2 meters, this point is critical to formation of uturn
            //now we have our start point
            vec3 start = new vec3(mf.gyd.rEast + Math.Sin(head) * 4, mf.gyd.rNorth + Math.Cos(head) * 4, head);
            vec3 goal = new vec3(0.0,0.0,0.0);

            //now we go the other way to turn round
            head -= Math.PI;
            if (head < 0) head += glm.twoPI;

            //set up the goal point for Dubins
            goal.heading = head;
            if (isTurnRight)
            {
                goal.easting = start.easting - (Math.Cos(-head) * turnOffset);
                goal.northing = start.northing - (Math.Sin(-head) * turnOffset);
            }
            else
            {
                goal.easting = start.easting + (Math.Cos(-head) * turnOffset);
                goal.northing = start.northing + (Math.Sin(-head) * turnOffset);
            }

            //generate the turn points
            ytList = dubYouTurnPath.GenerateDubins(start, goal);

            mf.guidanceLookPos.easting = ytList[ytList.Count - 1].easting;
            mf.guidanceLookPos.northing = ytList[ytList.Count - 1].northing;

            //vec3 pt;
            //for (double a = 0; a < 2; a += 0.2)
            //{
            //    pt.easting = ytList[0].easting + (Math.Sin(head) * a);
            //    pt.northing = ytList[0].northing + (Math.Cos(head) * a);
            //    pt.heading = ytList[0].heading;
            //    ytList.Insert(0, pt);
            //}

            //int count = ytList.Count;

            //for (double i = 0.2; i <= 7; i += 0.2)
            //{
            //    pt.easting = ytList[count - 1].easting + (Math.Sin(head) * i);
            //    pt.northing = ytList[count - 1].northing + (Math.Cos(head) * i);
            //    pt.heading = head;
            //    ytList.Add(pt);
            //}


            mf.ABLine.isABValid = false;
            mf.curve.isCurveValid = false;
        }

        public int onA;

        //Duh.... What does this do....
        public void DrawYouTurn()
        {
            int ptCount = ytList.Count;
            if (ptCount < 3) return;
            GL.PointSize(mf.ABLine.lineWidth);

            if (isYouTurnTriggered)
                GL.Color3(0.95f, 0.5f, 0.95f);
            else if (isOutOfBounds)
                GL.Color3(0.9495f, 0.395f, 0.325f);
            else
                GL.Color3(0.395f, 0.925f, 0.30f);

            GL.Begin(PrimitiveType.Points);
            for (int i = 0; i < ptCount; i++)
            {
                GL.Vertex3(ytList[i].easting, ytList[i].northing, 0);
            }
            GL.End();
        }
    }
}