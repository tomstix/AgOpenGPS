using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;
using System.Threading;

namespace AgOpenGPS
{
    public class CYouTurn
    {
        //copy of the mainform address
        private readonly FormGPS mf;

        /// <summary>/// triggered right after youTurnTriggerPoint is set /// </summary>
        public bool isYouTurnTriggered;

        /// <summary>  /// turning right or left?/// </summary>
        public bool isYouTurnRight;

        /// <summary> /// Is the youturn button enabled? /// </summary>
        public bool isYouTurnBtnOn;

        public int rowSkipsWidth = 1, uTurnSmoothing = 10;

        public bool alternateSkips = false, previousBigSkip = true;
        public int rowSkipsWidth2 = 3, turnSkips = 2;

        /// <summary>  /// distance from headland as offset where to start turn shape /// </summary>
        public int youTurnStartOffset;

        //guidance values
        public double uturnDistanceFromBoundary, turnOffset;

        public bool isTurnCreationTooClose = false, isTurnCreationNotCrossingError = false;

        //list of points for scaled and rotated YouTurn line, used for pattern, dubins, abcurve, abline
        public List<vec3> ytList = new List<vec3>();

        //is UTurn pattern in or out of bounds
        public bool isOutOfBounds = false;

        //sequence of operations of finding the next turn 0 to 3
        public int youTurnPhase = -1;

        vec6 exitPoint = new vec6(0, 0, double.MaxValue, -1, -1, -1);
        vec6 entryPoint = new vec6(0, 0, double.MaxValue, -1, -1, -1);

        private List<vec3> offsetList = new List<vec3>();
        public bool TurnRight, SwapYouTurn = true;
        public int YouTurnType = 2, YouTurnTypeA = 0;

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

        public bool BuildDubinsYouTurn(bool isTurnRight, bool isHeadingSameWay, double howManyPathsAway, ref List<vec3> curList, ref List<vec3> refList)
        {
            int Count = isHeadingSameWay ? 1 : -1;

            if (youTurnPhase == 0)
            {
                ytList.Clear();

                exitPoint = new vec6(0, 0, double.MaxValue, -1, -1, -1);

                vec3 Start = new vec3(mf.gyd.rEast, mf.gyd.rNorth, 0);


                //currentLocationIndex = isHeadingSameWay ? pB = 1 : pA = 0; //pB highest


                for (int i = mf.gyd.currentLocationIndex; i >= 0 && i < curList.Count; i += Count)
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
                                if (DanielP.GetLineIntersection(Start, curList[i], mf.turn.turnArr[j].turnLine[l], mf.turn.turnArr[j].turnLine[k], out vec2 _Crossing, out double Time))
                                {
                                    if (Time < exitPoint.time)
                                        exitPoint = new vec6(_Crossing.easting, _Crossing.northing, Time, j, i, l);//i = point after the crossing
                                }
                                k = l;
                            }
                        }
                    }

                    if (exitPoint.boundaryIndex >= 0)
                    {
                        mf.distancePivotToTurnLine = glm.Distance(mf.pivotAxlePos, exitPoint.easting, exitPoint.northing);

                        break;
                    }
                    Start = curList[i];
                }

                if (exitPoint.boundaryIndex == -1)
                {
                    //Full emergency stop code goes here, it thinks its auto turn, but its not!
                    isTurnCreationNotCrossingError = true;
                    mf.distancePivotToTurnLine = -3333;
                    ResetCreatedYouTurn();
                    return false;
                }

                youTurnPhase++;
            }
            else if (youTurnPhase == 1)
            {
                offsetList.Clear();
                double distAway = (mf.tool.toolWidth - mf.tool.toolOverlap) * (howManyPathsAway + rowSkipsWidth * (isHeadingSameWay ? (isTurnRight ? 1 : -1) : (isTurnRight ? -1 : 1))) + (isHeadingSameWay ? mf.tool.toolOffset : -mf.tool.toolOffset);

                double distSqAway = (distAway * distAway) - 0.01;

                for (int i = 0; i < refList.Count; i++)
                {
                    vec3 point = new vec3(
                    refList[i].easting + (Math.Sin(glm.PIBy2 + refList[i].heading) * distAway),
                    refList[i].northing + (Math.Cos(glm.PIBy2 + refList[i].heading) * distAway),
                    refList[i].heading);
                    bool Add = true;
                    for (int t = 0; t < refList.Count; t++)
                    {
                        double dist = ((point.easting - refList[t].easting) * (point.easting - refList[t].easting))
                            + ((point.northing - refList[t].northing) * (point.northing - refList[t].northing));
                        if (dist < distSqAway)
                        {
                            Add = false;
                            break;
                        }
                    }
                    if (Add)
                    {
                        if (offsetList.Count > 0)
                        {
                            double dist = ((point.easting - offsetList[offsetList.Count - 1].easting) * (point.easting - offsetList[offsetList.Count - 1].easting))
                                + ((point.northing - offsetList[offsetList.Count - 1].northing) * (point.northing - offsetList[offsetList.Count - 1].northing));
                            if (dist > 1)
                                offsetList.Add(point);
                        }
                        else offsetList.Add(point);
                    }
                }
                youTurnPhase++;
            }
            else if (youTurnPhase == 2)
            {
                turnOffset = ((mf.tool.toolWidth - mf.tool.toolOverlap) * rowSkipsWidth) + (isTurnRight ? mf.tool.toolOffset * 2.0 : -mf.tool.toolOffset * 2.0);

                TurnRight = (turnOffset < 0) ? !isTurnRight : isTurnRight;
                //if (exitPoint.boundaryIndex != 0) TurnRight = !TurnRight;

                int Idx = (exitPoint.turnLineIdx - (TurnRight ? 0 : 1)).Clamp(mf.turn.turnArr[exitPoint.boundaryIndex].turnLine.Count);

                int StartInt = Idx;

                int Count2 = TurnRight ? 1 : -1;

                ytList.Add(new vec3(exitPoint.easting, exitPoint.northing, curList[exitPoint.crossingIdx - Count].heading));

                vec3 Start = new vec3(exitPoint.easting, exitPoint.northing, curList[exitPoint.crossingIdx - Count].heading);

                entryPoint = new vec6(0, 0, double.MaxValue, -2, -2, -2);
                bool Loop = true;
                //check if outside a border
                for (int j = Idx; (TurnRight ? j < StartInt : j > StartInt) || Loop; j += Count2)
                {
                    if (j >= mf.turn.turnArr[exitPoint.boundaryIndex].turnLine.Count || j < 0)
                    {
                        if (j < 0)
                            j = mf.turn.turnArr[exitPoint.boundaryIndex].turnLine.Count - 1;
                        else j = 0;
                        Loop = false;
                    }
                    vec3 End = mf.turn.turnArr[exitPoint.boundaryIndex].turnLine[j];

                    int L = 0;
                    for (int K = 1; K < offsetList.Count; L = K++)
                    {
                        if (DanielP.GetLineIntersection(Start, End, offsetList[L], offsetList[K], out vec2 _Crossing, out double Time) && Time < entryPoint.time)
                            entryPoint = new vec6(_Crossing.easting, _Crossing.northing, Time, 1, isHeadingSameWay ? K : L, j);
                    }

                    bool selfintersect = false;

                    vec2 End2;
                    vec2 Start2 = new vec2(exitPoint.easting, exitPoint.northing);
                    for (int i = exitPoint.crossingIdx; i >= 0 && i < curList.Count; i += Count)
                    {
                        End2 = curList[i];
                        if (DanielP.GetLineIntersection(Start, End, Start2, End2, out vec2 _Crossing, out double Time) && Time < entryPoint.time)
                            entryPoint = new vec6(_Crossing.easting, _Crossing.northing, Time, -1, i, j);
                        Start2 = End2;
                    }

                    Start2 = new vec2(exitPoint.easting, exitPoint.northing);
                    for (int i = exitPoint.crossingIdx - Count; i >= 0 && i < curList.Count; i -= Count)
                    {
                        End2 = curList[i];
                        if (DanielP.GetLineIntersection(Start, End, Start2, End2, out _, out double Time) && Time < entryPoint.time)
                        {
                            entryPoint.time = Time;
                            selfintersect = true;
                        }
                        Start2 = End2;
                    }

                    if (selfintersect || entryPoint.crossingIdx >= 0)
                        break;//we only care for the closest one;
                    else ytList.Add(End);

                    Start = End;
                }

                if (entryPoint.crossingIdx == -2)
                {
                    //Full emergency stop code goes here, it thinks its auto turn, but its not!
                    isTurnCreationNotCrossingError = true;
                    mf.distancePivotToTurnLine = -3333;
                    ResetCreatedYouTurn();
                    return false;
                }

                ytList.Add(new vec3(entryPoint.easting, entryPoint.northing, 0.0));
                youTurnPhase++;
            }
            else if (youTurnPhase == 3)
            {
                if (entryPoint.boundaryIndex < 0)
                    SwapYouTurn = false;
                else
                    SwapYouTurn = true;

                YouTurnTypeA = YouTurnType;

                if (true || (YouTurnTypeA > 0 && Math.Abs(turnOffset) > mf.vehicle.minTurningRadius * 2) || entryPoint.boundaryIndex < 0)
                {
                    List<vec3> test2 = new List<vec3>();
                    List<vec3> ExitLine, EntryLine;

                    if (isHeadingSameWay)
                    {
                        ExitLine = curList.GetRange(0, exitPoint.crossingIdx);

                        if (entryPoint.boundaryIndex > 0)
                            EntryLine = offsetList.GetRange(0, entryPoint.crossingIdx);
                        else
                            EntryLine = curList.GetRange(entryPoint.crossingIdx, curList.Count - entryPoint.crossingIdx);
                        EntryLine.Reverse();
                    }
                    else
                    {
                        ExitLine = curList.GetRange(exitPoint.crossingIdx - Count, curList.Count - (exitPoint.crossingIdx - Count));
                        ExitLine.Reverse();

                        if (entryPoint.boundaryIndex > 0)
                            EntryLine = offsetList.GetRange(entryPoint.crossingIdx - Count, offsetList.Count - (entryPoint.crossingIdx - Count));
                        else
                            EntryLine = curList.GetRange(0, entryPoint.crossingIdx - Count);
                    }

                    if (entryPoint.boundaryIndex > 0 && YouTurnTypeA == 1)
                    {
                        double Dx = entryPoint.northing - exitPoint.northing;
                        double Dy = entryPoint.easting - exitPoint.easting;

                        double Distance, MaxDistance = 0;

                        for (int j = 0; j < ytList.Count; j++)
                        {
                            Distance = ((Dx * ytList[j].easting) - (Dy * ytList[j].northing) + (entryPoint.easting
                                        * exitPoint.northing) - (entryPoint.northing * exitPoint.easting))
                                            / Math.Sqrt((Dx * Dx) + (Dy * Dy));
                            if (!TurnRight && Distance < MaxDistance || TurnRight && Distance > MaxDistance)
                            {
                                MaxDistance = Distance;
                            }
                        }

                        ytList.Clear();

                        if (Math.Abs(MaxDistance) > 0.01)
                        {
                            double Heading = Math.Atan2(Dy, Dx);

                            double Hcos = Math.Cos(Heading);
                            double Hsin = Math.Sin(Heading);
                            vec2 Start = new vec2(entryPoint.northing + Hsin * -MaxDistance - Hcos * mf.maxCrossFieldLength,
                                                  entryPoint.easting + Hcos * MaxDistance - Hsin * mf.maxCrossFieldLength);

                            vec2 End = new vec2(exitPoint.northing + Hsin * -MaxDistance + Hcos * mf.maxCrossFieldLength,
                                                exitPoint.easting + Hcos * MaxDistance + Hsin * mf.maxCrossFieldLength);

                            vec2 End2, Start2 = new vec2(exitPoint.easting, exitPoint.northing);

                            for (int j = ExitLine.Count - 1; j >= 0; j--)
                            {
                                End2 = ExitLine[j];

                                if (DanielP.GetLineIntersection(Start, End, Start2, End2, out vec2 _Crossing, out double Time))
                                {
                                    ytList.Add(new vec3(_Crossing.easting, _Crossing.northing, 0.0));
                                    break;
                                }
                                else
                                {
                                    ExitLine.RemoveAt(j);
                                }
                                Start2 = End2;
                            }

                            Start2 = new vec2(entryPoint.easting, entryPoint.northing);
                            for (int j = EntryLine.Count - 1; j >= 0; j--)
                            {
                                End2 = EntryLine[j];
                                if (DanielP.GetLineIntersection(Start, End, Start2, End2, out vec2 _Crossing, out double Time))
                                {
                                    ytList.Add(new vec3(_Crossing.easting, _Crossing.northing, 0.0));
                                    break;
                                }
                                else
                                {
                                    EntryLine.RemoveAt(j);
                                }
                                Start2 = End2;
                            }
                        }
                        else
                        {
                            ytList.Add(new vec3(exitPoint.easting, exitPoint.northing, 0.0));
                            ytList.Add(new vec3(entryPoint.easting, entryPoint.northing, 0.0));
                        }
                    }

                    test2.AddRange(ExitLine);
                    test2.AddRange(ytList);
                    test2.AddRange(EntryLine);

                    double a = TurnRight ? mf.vehicle.minTurningRadius : Math.Max(mf.vehicle.minTurningRadius, uturnDistanceFromBoundary);
                    double b = TurnRight ? Math.Max(mf.vehicle.minTurningRadius, uturnDistanceFromBoundary) : mf.vehicle.minTurningRadius;

                    List<vec2> test = new List<vec2>();
                    test.AddRange(test2.ToArray());

                    bool done = test.CalculateRoundedCorner(a, b, false, 0.0436332, CancellationToken.None, ExitLine.Count, test.Count - EntryLine.Count);

                    if (!done || test.Count == 0) YouTurnTypeA = 0;
                    else
                    {
                        int i = 0;
                        for (; test.Count > 0 && i < ExitLine.Count; i++)
                        {
                            if (test[0].northing == ExitLine[i].northing && test[0].easting == ExitLine[i].easting)
                            {
                                test.RemoveAt(0);
                            }
                            else break;
                        }

                        //now stepback to add uturn length again? . . .
                        double TotalDist = 0;
                        i--;
                        for (; i >= 0; i--)
                        {
                            double Distance = glm.Distance(test[0], ExitLine[i]);
                            if (TotalDist + Distance > youTurnStartOffset)
                            {
                                double Dx = (test[0].northing - ExitLine[i].northing) / Distance * (youTurnStartOffset - TotalDist);
                                double Dy = (test[0].easting - ExitLine[i].easting) / Distance * (youTurnStartOffset - TotalDist);
                                test.Insert(0, new vec3(test[0].easting - Dy, test[0].northing - Dx, 0.0));
                                break;
                            }
                            else
                            {
                                TotalDist += Distance;
                                test.Insert(0, ExitLine[i]);
                            }
                        }

                        i = EntryLine.Count - 1;

                        for (int j = test.Count - 1; i >= 0; i--)
                        {
                            if (test[j].northing == EntryLine[i].northing && test[j].easting == EntryLine[i].easting)
                            {
                                test.RemoveAt(j);
                                j = test.Count - 1;
                            }
                            else
                            {
                                break;
                            }
                        }

                        //now stepback to add uturn length again? . . .
                        TotalDist = 0;
                        i++;

                        for (; i < EntryLine.Count; i++)
                        {
                            double Distance = glm.Distance(test[test.Count - 1], EntryLine[i]);
                            if (TotalDist + Distance > youTurnStartOffset)
                            {
                                double Dx = (test[test.Count - 1].northing - EntryLine[i].northing) / Distance;
                                double Dy = (test[test.Count - 1].easting - EntryLine[i].easting) / Distance;
                                test.Add(new vec3(test[test.Count - 1].easting - Dy * (youTurnStartOffset - TotalDist), test[test.Count - 1].northing - Dx * (youTurnStartOffset - TotalDist), 0.0));

                                test.Add(new vec3(test[test.Count - 1].easting - Dy, test[test.Count - 1].northing - Dx, 0.0));

                                break;
                            }
                            else
                            {
                                TotalDist += Distance;
                                test.Add(EntryLine[i]);
                            }
                        }

                        ytList.Clear();
                        double heading = 0;
                        for (int j = 0; j < test.Count; j++)
                        {
                            if (j + 1 < test.Count)
                            {
                                double dx1 = test[j + 1].northing - test[j].northing;
                                double dy1 = test[j + 1].easting - test[j].easting;
                                double length = DanielP.GetLength(dx1, dy1);
                                double num = Math.Round(length, MidpointRounding.AwayFromZero);
                                if (num > 1)
                                {
                                    dx1 /= num;
                                    dy1 /= num;

                                    for (int k = 1; k <= num; k++)
                                    {
                                        test.Insert(j + k, new vec3(test[j].easting + dy1 * k, test[j].northing + dx1 * k, 0));
                                    }
                                }
                                heading = Math.Atan2(test[j + 1].easting - test[j].easting, test[j + 1].northing - test[j].northing);
                            }
                            if (heading < 0) heading += glm.twoPI;

                            ytList.Add(new vec3(test[j].easting, test[j].northing, heading));
                        }

                        if (ytList.Count == 0) return false;
                        else youTurnPhase = 10;
                    }
                }
                else YouTurnTypeA = 0;

                if (YouTurnTypeA == 0)
                {
                    int Index1 = exitPoint.crossingIdx - Count;
                    int Index2 = entryPoint.crossingIdx - Count;

                    int count = isHeadingSameWay ? 1 : -1;

                    double Dx = exitPoint.northing - curList[Index1].northing;
                    double Dy = exitPoint.easting - curList[Index1].easting;

                    double Heading3 = Math.Atan2(Dy, Dx);

                    vec3 Point1 = new vec3(exitPoint.easting, exitPoint.northing, Heading3);
                    vec3 Point2 = new vec3(entryPoint.easting, entryPoint.northing, Heading3 + Math.PI);

                    double Offset = 5;

                    bool Finalize = false;
                    CDubins dubYouTurnPath = new CDubins();

                    while (true)
                    {
                        ytList.Clear();

                        ytList = dubYouTurnPath.GenerateDubins(Point1, Point2);

                        isOutOfBounds = false;
                        for (int j = 0; j < ytList.Count; j += 2)
                        {
                            if (!mf.turn.turnArr[0].IsPointInTurnWorkArea(ytList[j])) isOutOfBounds = true;
                            if (isOutOfBounds) break;

                            for (int l = 1; l < mf.bnd.bndArr.Count; l++)
                            {
                                //make sure not inside a non drivethru boundary
                                if (mf.bnd.bndArr[l].isDriveThru || mf.bnd.bndArr[l].isDriveAround) continue;
                                if (mf.turn.turnArr[l].IsPointInTurnWorkArea(ytList[j]))
                                {
                                    isOutOfBounds = true;
                                    break;
                                }
                            }
                            if (isOutOfBounds) break;
                        }

                        if (!isOutOfBounds && Offset == 5)
                        {
                            Offset = 0.5;
                            count = isHeadingSameWay ? -1 : 1;
                            Index2 -= count;
                            Index1 -= count;
                        }
                        else if (isOutOfBounds && Offset == 0.5)
                        {
                            Offset = 0.05;
                            count = isHeadingSameWay ? 1 : -1;
                            Index2 -= count;
                            Index1 -= count;
                        }
                        else if (!isOutOfBounds && Offset == 0.05)
                        {
                            Offset = youTurnStartOffset;
                            //count = isHeadingSameWay ? -1 : 1;
                            //Index2 -= count;
                            //Index1 -= count;
                            Finalize = true;
                        }

                        double distSoFar = 0;
                        //cycle thru segments and keep adding lengths. check if start and break if so.
                        while (true)
                        {
                            if (Index2 == -1 || Index2 >= offsetList.Count) return false;

                            double dx1 = Point2.northing - offsetList[Index2].northing;
                            double dy1 = Point2.easting - offsetList[Index2].easting;
                            double length1 = DanielP.GetLength(dx1, dy1);

                            //will we go too far?
                            if ((length1 + distSoFar) > Offset)
                            {
                                double factor = (Offset - distSoFar) / length1;

                                Point2.easting -= dy1 * factor;
                                Point2.northing -= dx1 * factor;
                                if (Finalize)
                                    ytList.Add(new vec3(Point2));
                                break; //tempDist contains the full length of next segment
                            }
                            distSoFar += length1;
                            Point2.easting = offsetList[Index2].easting;
                            Point2.northing = offsetList[Index2].northing;
                            Point2.heading = isHeadingSameWay ? offsetList[Index2].heading + Math.PI : offsetList[Index2].heading;

                            if (Finalize)
                                ytList.Add(new vec3(Point2));
                            Index2 -= count;
                        }
                        distSoFar = 0;
                        //cycle thru segments and keep adding lengths. check if start and break if so.
                        while (true)
                        {
                            if (Index1 == -1 || Index1 >= curList.Count) return false;

                            double dx1 = Point1.northing - curList[Index1].northing;
                            double dy1 = Point1.easting - curList[Index1].easting;
                            double length1 = DanielP.GetLength(dx1, dy1);


                            //will we go too far?
                            if ((length1 + distSoFar) > Offset)
                            {
                                double factor = (Offset - distSoFar) / length1;

                                Point1.easting -= dy1 * factor;
                                Point1.northing -= dx1 * factor;
                                if (Finalize)
                                    ytList.Insert(0, new vec3(Point1));
                                break; //tempDist contains the full length of next segment
                            }
                            distSoFar += length1;
                            Point1.easting = curList[Index1].easting;
                            Point1.northing = curList[Index1].northing;
                            Point1.heading = isHeadingSameWay ? curList[Index1].heading : curList[Index1].heading + Math.PI;

                            if (Finalize)
                                ytList.Insert(0, new vec3(Point1));
                            Index1 -= count;
                        }

                        double distancePivotToTurnLine;
                        for (int j = 0; j < ytList.Count; j += 2)
                        {
                            distancePivotToTurnLine = glm.Distance(ytList[j], mf.pivotAxlePos);
                            if (distancePivotToTurnLine > 3)
                            {
                                isTurnCreationTooClose = false;
                            }
                            else
                            {
                                isTurnCreationTooClose = true;
                                break;
                            }
                        }

                        if (Finalize)
                        {
                            double heading = 0;
                            for (int j = 0; j < ytList.Count; j++)
                            {
                                if (j + 1 < ytList.Count)
                                {
                                    double dx1 = ytList[j + 1].northing - ytList[j].northing;
                                    double dy1 = ytList[j + 1].easting - ytList[j].easting;
                                    double length = DanielP.GetLength(dx1, dy1);
                                    double num = Math.Round(length, MidpointRounding.AwayFromZero);
                                    if (num > 1)
                                    {
                                        dx1 /= num;
                                        dy1 /= num;

                                        for (int k = 1; k <= num; k++)
                                        {
                                            ytList.Insert(j + k, new vec3(ytList[j].easting + dy1 * k, ytList[j].northing + dx1 * k, ytList[j].heading));
                                        }
                                    }
                                    heading = Math.Atan2(ytList[j + 1].easting - ytList[j].easting, ytList[j + 1].northing - ytList[j].northing);
                                }
                                if (heading < 0) heading += glm.twoPI;

                                ytList[j].heading = heading;
                            }

                            double ytLength = 0;
                            for (int j = 0; j + 2 < ytList.Count; j++)
                            {
                                ytLength += glm.Distance(ytList[j], ytList[j + 1]);
                            }

                            youTurnPhase = 10;
                            return true;
                        }
                    }
                }
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

            if (SwapYouTurn)
            {
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
            SwapYouTurn = true;
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
            SwapYouTurn = true;
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
            GL.Color3(1.0f, 0.0f, 1.0f);
            GL.PointSize(mf.ABLine.lineWidth*4);
            GL.Begin(PrimitiveType.Points);
            GL.Vertex3(entryPoint.easting, entryPoint.northing, 0);
            GL.Vertex3(exitPoint.easting, exitPoint.northing, 0);
            GL.End();

            GL.Color3(1.0f, 1.0f, 1.0f);
            GL.Begin(PrimitiveType.LineStrip);
            for (int i = 0; i < offsetList.Count; i++)
            {
                GL.Vertex3(offsetList[i].easting, offsetList[i].northing, 0);
            }
            GL.End();

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