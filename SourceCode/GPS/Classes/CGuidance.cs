using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CGuidance
    {
        private readonly FormGPS mf;

        //steer, pivot, and ref indexes
        private int sA, sB, pA, pB;

        public double distanceFromCurrentLineSteer, distanceFromCurrentLinePivot;
        public double steerAngle, ppRadius, rEast, rNorth;

        public double inty, xTrackSteerCorrection = 0;
        public double steerHeadingError, steerHeadingErrorDegrees;

        public double distSteerError, lastDistSteerError, derivativeDistError;

        public double pivotDistanceError, UturnHeading;

        public vec2 goalPoint = new vec2(0, 0);
        public vec2 radiusPoint = new vec2(0, 0);
        public bool isLateralTriggered;
        //derivative counter
        public int counter, currentLocationIndex;

        public CGuidance(FormGPS _f)
        {
            //constructor
            mf = _f;
        }

        private void DoSteerAngleCalc()
        {
            if (mf.isReverse) steerHeadingError *= -1;
            //Overshoot setting on Stanley tab
            steerHeadingError *= mf.vehicle.stanleyHeadingErrorGain;

            double sped = Math.Abs(mf.avgSpeed);
            if (sped > 1) sped = 1 + 0.277 * (sped - 1);
            else sped = 1;
            double XTEc = Math.Atan((distanceFromCurrentLineSteer * mf.vehicle.stanleyDistanceErrorGain)
                / (sped));

            xTrackSteerCorrection = (xTrackSteerCorrection * 0.5) + XTEc * (0.5);

            steerAngle = glm.toDegrees((xTrackSteerCorrection + steerHeadingError) * -1.0);

            if (!mf.yt.isYouTurnTriggered)
            {
                if (Math.Abs(distanceFromCurrentLineSteer) > 0.5) steerAngle *= 0.5;
                else steerAngle *= (1 - Math.Abs(distanceFromCurrentLineSteer));

                //integral slider is set to 0
                if (mf.vehicle.stanleyIntegralGainAB != 0 || mf.isReverse)
                {
                    //derivative of steer distance error
                    distSteerError = (distSteerError * 0.95) + ((xTrackSteerCorrection * 60) * 0.05);
                    if (counter++ > 5)
                    {
                        derivativeDistError = distSteerError - lastDistSteerError;
                        lastDistSteerError = distSteerError;
                        counter = 0;
                    }

                    pivotDistanceError = (pivotDistanceError * 0.6) + (distanceFromCurrentLinePivot * 0.4);

                    if (mf.pn.speed > mf.startSpeed
                        && mf.isAutoSteerBtnOn
                        && Math.Abs(derivativeDistError) < 1
                        && Math.Abs(pivotDistanceError) < 0.25)
                    {
                        //if over the line heading wrong way, rapidly decrease integral
                        if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                        {
                            inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.1;
                        }
                        else
                        {
                            inty += pivotDistanceError * mf.vehicle.stanleyIntegralGainAB * -0.01;
                        }

                        if (inty > 5.0) inty = 5.0;
                        else if (inty < -5.0) inty = -5.0;
                    }
                    else inty *= 0.7;
                }
                else inty = 0;
            }

            if (steerAngle < -mf.vehicle.maxSteerAngle) steerAngle = -mf.vehicle.maxSteerAngle;
            else if (steerAngle > mf.vehicle.maxSteerAngle) steerAngle = mf.vehicle.maxSteerAngle;

            //Convert to millimeters from meters
            mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
            mf.guidanceLineSteerAngle = (short)(steerAngle * 100);
        }

        /// <summary>
        /// Find the steer angle for closest point in list, curvature and integral
        /// </summary>
        public void StanleyGuidance(vec3 pivot, vec3 steer, ref List<vec3> curList, bool _isHeadingSameWay)
        {
            bool ResetUturn = false;
            List<vec3> Points = mf.yt.isYouTurnTriggered ? mf.yt.ytList : curList;
            bool isHeadingSameWay = mf.yt.isYouTurnTriggered || _isHeadingSameWay;

            //find the closest point roughly
            int cc = 0, dd = Points.Count;
            int ptCount = Points.Count;
            if (ptCount > 1)
            {
                double minDistA = double.MaxValue;
                double minDistB = double.MaxValue;

                if (ptCount > 20)
                {
                    for (int j = 0; j < ptCount; j += 10)
                    {
                        double dist = ((steer.easting - Points[j].easting) * (steer.easting - Points[j].easting))
                                        + ((steer.northing - Points[j].northing) * (steer.northing - Points[j].northing));
                        if (dist < minDistA)
                        {
                            minDistA = dist;
                            cc = j;
                        }
                    }

                    minDistA = minDistB = double.MaxValue;

                    dd = cc + 7;
                    if (dd > ptCount) dd = ptCount;
                    cc -= 7;
                    if (cc < 0) cc = 0;
                }

                //find the closest 2 points to current close call
                for (int j = cc; j < dd; j++)
                {
                    double dist = ((steer.easting - Points[j].easting) * (steer.easting - Points[j].easting))
                                    + ((steer.northing - Points[j].northing) * (steer.northing - Points[j].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        sB = sA;
                        minDistA = dist;
                        sA = j;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        sB = j;
                    }
                }

                //just need to make sure the points continue ascending or heading switches all over the place
                if (sA > sB) { int C = sA; sA = sB; sB = C; }

                minDistA = minDistB = double.MaxValue;

                if (isHeadingSameWay)
                {
                    dd = sB+1; cc = dd - 12;
                    if (cc < 0) cc = 0;
                }
                else
                {
                    cc = sA; dd = sA + 12;
                    if (dd > ptCount) dd = ptCount;
                }

                //find the closest 2 points of pivot back from steer
                for (int j = cc; j < dd; j++)
                {
                    double dist = ((pivot.easting - Points[j].easting) * (pivot.easting - Points[j].easting))
                                    + ((pivot.northing - Points[j].northing) * (pivot.northing - Points[j].northing));
                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        pB = pA;
                        minDistA = dist;
                        pA = j;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        pB = j;
                    }
                }

                //just need to make sure the points continue ascending or heading switches all over the place
                if (pA > pB) { int C = pA; pA = pB; pB = C; }

                if (mf.yt.isYouTurnTriggered)
                {
                    //feed backward to turn slower to keep pivot on
                    sA -= 7;
                    if (sA < 0)
                    {
                        sA = 0;
                    }
                    sB = sA + 1;

                    //return and reset if too far away or end of the line
                    if (minDistA > 16 || sB >= ptCount - 8)
                    {
                        ResetUturn = true;
                    }
                }

                //get the pivot distance from currently active AB segment   ///////////  Pivot  ////////////
                double dx = Points[pB].easting - Points[pA].easting;
                double dy = Points[pB].northing - Points[pA].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

                //how far from current AB Line is fix
                distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (Points[pB].easting
                            * Points[pA].northing) - (Points[pB].northing * Points[pA].easting))
                                / Math.Sqrt((dy * dy) + (dx * dx));

                if (!isHeadingSameWay)
                    distanceFromCurrentLinePivot *= -1.0;

                double U = (((pivot.easting - Points[pA].easting) * dx)
                                + ((pivot.northing - Points[pA].northing) * dy))
                                / ((dx * dx) + (dy * dy));

                UturnHeading = Points[pA].heading;
                rEast = Points[pA].easting + (U * dx);
                rNorth = Points[pA].northing + (U * dy);
                currentLocationIndex = isHeadingSameWay ? pB : pA;

                ////////// steer ///////////// get the distance from currently active AB segment of steer axle 
                dx = Points[sB].easting - Points[sA].easting;
                dy = Points[sB].northing - Points[sA].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

                //how far from current AB Line is fix
                distanceFromCurrentLineSteer = ((dy * steer.easting) - (dx * steer.northing) + (Points[sB].easting
                            * Points[sA].northing) - (Points[sB].northing * Points[sA].easting))
                                / Math.Sqrt((dy * dy) + (dx * dx));

                if (!isHeadingSameWay)
                    distanceFromCurrentLineSteer *= -1.0;
                //create the integral offset
                if (!mf.yt.isYouTurnTriggered && inty != 0)
                    distanceFromCurrentLineSteer -= inty;

                steerHeadingError = steer.heading - Points[sA].heading;

                //Fix the circular error
                while (steerHeadingError > glm.PIBy2)
                    steerHeadingError -= Math.PI;
                while (steerHeadingError < -glm.PIBy2)
                    steerHeadingError += Math.PI;

                DoSteerAngleCalc();
            }
            else
            {
                //invalid distance so tell AS module
                mf.guidanceLineDistanceOff = 32000;
                ResetUturn = true;
            }
            if (ResetUturn) mf.yt.ResetYouTurn();
        }

        public void PurePursuitGuidance(vec3 pivot, ref List<vec3> curList, bool _isHeadingSameWay)
        {
            bool ResetUturn = false;
            List<vec3> Points = mf.yt.isYouTurnTriggered ? mf.yt.ytList : curList;
            bool isHeadingSameWay = mf.yt.isYouTurnTriggered || _isHeadingSameWay;

            int ptCount = Points.Count;
            if (ptCount > 1)
            {
                double dist, dx, dy;
                double minDistA = double.MaxValue, minDistB = double.MaxValue;

                //find the closest 2 points to current fix
                for (int t = 0; t < Points.Count; t++)
                {
                    dist = glm.DistanceSquared(pivot, Points[t]);

                    if (dist < minDistA)
                    {
                        minDistB = minDistA;
                        pB = pA;
                        minDistA = dist;
                        pA = t;
                    }
                    else if (dist < minDistB)
                    {
                        minDistB = dist;
                        pB = t;
                    }
                }

                if (mf.yt.isYouTurnTriggered)
                {
                    mf.yt.onA = Points.Count / 2;
                    if (pA < mf.yt.onA)
                    {
                        mf.yt.onA = -pA;
                    }
                    else
                    {
                        mf.yt.onA = Points.Count - pA;
                    }

                    if (pA > pB) { int C = pA; pA = pB; pB = C; }

                    //return and reset if too far away or end of the line
                    if (pB >= Points.Count - 1)
                    {
                        ResetUturn = true;
                    }
                }
                //just need to make sure the points continue ascending or heading switches all over the place
                else if (pA > pB) { int C = pA; pA = pB; pB = C; }

                if (mf.recPath.isFollowingRecPath || mf.recPath.isFollowingDubinsToPath || mf.recPath.isFollowingDubinsHome)
                {
                    mf.recPath.C = pA;
                    if (mf.recPath.isFollowingRecPath && pB >= ptCount-1)
                        mf.recPath.isEndOfTheRecLine = true;
                }
                //get the distance from currently active AB line
                dx = Points[pB].easting - Points[pA].easting;
                dy = Points[pB].northing - Points[pA].northing;

                if (Math.Abs(dx) < Double.Epsilon && Math.Abs(dy) < Double.Epsilon) return;

                //abHeading = Math.Atan2(dz, dx);
                double length = Math.Sqrt((dy * dy) + (dx * dx));
                //how far from current AB Line is fix
                distanceFromCurrentLinePivot = ((dy * pivot.easting) - (dx * pivot.northing) + (Points[pB].easting
                            * Points[pA].northing) - (Points[pB].northing * Points[pA].easting))
                                / length;

                if (isHeadingSameWay)
                    distanceFromCurrentLinePivot *= -1.0;

                if (!mf.yt.isYouTurnTriggered)
                {
                    //integral slider is set to 0
                    if (mf.vehicle.purePursuitIntegralGain != 0 || mf.isReverse)
                    {
                        pivotDistanceError = pivotDistanceError * 0.8 + distanceFromCurrentLinePivot * 0.2;

                        if (counter++ > 4)
                        {
                            derivativeDistError = pivotDistanceError - lastDistSteerError;
                            lastDistSteerError = pivotDistanceError;
                            counter = 0;
                            derivativeDistError *= 2;
                        }

                        if (mf.isAutoSteerBtnOn && mf.avgSpeed > 2.5 && Math.Abs(derivativeDistError) < 0.1)
                        {
                            //if over the line heading wrong way, rapidly decrease integral
                            if ((inty < 0 && distanceFromCurrentLinePivot < 0) || (inty > 0 && distanceFromCurrentLinePivot > 0))
                            {
                                inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.04;
                            }
                            else
                                inty += pivotDistanceError * mf.vehicle.purePursuitIntegralGain * -0.02;

                            if (inty > 5.0) inty = 5.0;
                            else if (inty < -5.0) inty = -5.0;
                        }
                        else inty *= 0.96;
                    }
                    else inty = 0;
                }

                double U = (((pivot.easting - Points[pA].easting) * dx)
                            + ((pivot.northing - Points[pA].northing) * dy))
                            / ((dx * dx) + (dy * dy));

                UturnHeading = Points[pA].heading;
                rEast = Points[pA].easting + (U * dx);
                rNorth = Points[pA].northing + (U * dy);

                //update base on autosteer settings and distance from line
                double goalPointDistance = mf.vehicle.UpdateGoalPointDistance() * (mf.yt.isYouTurnTriggered ? 0.8 : 1.0);

                bool ReverseHeading = mf.isReverse ? !isHeadingSameWay : isHeadingSameWay;

                currentLocationIndex = isHeadingSameWay ? pB : pA;

                int count = ReverseHeading ? 1 : -1;
                vec3 start = new vec3(rEast, rNorth, 0);
                double distSoFar = 0;

                for (int i = ReverseHeading ? pB : pA; i < Points.Count && i >= 0; i += count)
                {
                    // used for calculating the length squared of next segment.
                    double tempDist = glm.Distance(start, Points[i]);

                    //will we go too far?
                    if ((tempDist + distSoFar) > goalPointDistance)
                    {
                        double j = (goalPointDistance - distSoFar) / tempDist; // the remainder to yet travel

                        goalPoint.easting = (((1 - j) * start.easting) + (j * Points[i].easting));
                        goalPoint.northing = (((1 - j) * start.northing) + (j * Points[i].northing));
                        break;
                    }
                    else distSoFar += tempDist;
                    start = Points[i];

                    if (mf.yt.isYouTurnTriggered && i == Points.Count - 1)//goalPointDistance is longer than remaining u-turn
                    {
                        ResetUturn = true;
                    }
                }

                if (!mf.yt.isYouTurnTriggered)
                {
                    //create the integral offset
                    goalPoint.easting += ReverseHeading ? inty * (dy / length) : inty * -(dy / length);
                    goalPoint.northing += ReverseHeading ? inty * -(dx / length) : inty * (dx / length);
                }

                //calc "D" the distance from pivot axle to lookahead point
                double goalPointDistanceSquared = glm.DistanceSquared(goalPoint.northing, goalPoint.easting, pivot.northing, pivot.easting);

                double localHeading = glm.twoPI - mf.fixHeading;

                ppRadius = goalPointDistanceSquared / (2 * (((goalPoint.easting - pivot.easting) * Math.Cos(localHeading)) + ((goalPoint.northing - pivot.northing) * Math.Sin(localHeading))));

                steerAngle = glm.toDegrees(Math.Atan(2 * (((goalPoint.easting - pivot.easting) * Math.Cos(localHeading))
                    + ((goalPoint.northing - pivot.northing) * Math.Sin(localHeading))) * mf.vehicle.wheelbase / goalPointDistanceSquared));

                if (steerAngle < -mf.vehicle.maxSteerAngle) steerAngle = -mf.vehicle.maxSteerAngle;
                if (steerAngle > mf.vehicle.maxSteerAngle) steerAngle = mf.vehicle.maxSteerAngle;

                if (ppRadius < -500) ppRadius = -500;
                if (ppRadius > 500) ppRadius = 500;

                radiusPoint.easting = pivot.easting + (ppRadius * Math.Cos(localHeading));
                radiusPoint.northing = pivot.northing + (ppRadius * Math.Sin(localHeading));

                if (mf.ABLine.isBtnABLineOn && mf.isAngVelGuidance)
                {
                    //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
                    mf.setAngVel = 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(steerAngle))) / mf.vehicle.wheelbase;
                    mf.setAngVel = glm.toDegrees(mf.setAngVel) * 100;

                    //clamp the steering angle to not exceed safe angular velocity
                    if (Math.Abs(mf.setAngVel) > 1000)
                    {
                        //mf.setAngVel = mf.setAngVel < 0 ? -mf.vehicle.maxAngularVelocity : mf.vehicle.maxAngularVelocity;
                        mf.setAngVel = mf.setAngVel < 0 ? -1000 : 1000;
                    }
                }
                else
                {
                    //angular velocity in rads/sec  = 2PI * m/sec * radians/meters
                    double angVel = glm.twoPI * 0.277777 * mf.pn.speed * (Math.Tan(glm.toRadians(steerAngle))) / mf.vehicle.wheelbase;

                    //clamp the steering angle to not exceed safe angular velocity
                    if (Math.Abs(angVel) > mf.vehicle.maxAngularVelocity)
                    {
                        steerAngle = glm.toDegrees(steerAngle > 0 ?
                                (Math.Atan((mf.vehicle.wheelbase * mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.avgSpeed * 0.277777)))
                            : (Math.Atan((mf.vehicle.wheelbase * -mf.vehicle.maxAngularVelocity) / (glm.twoPI * mf.avgSpeed * 0.277777))));
                    }
                }

                //Convert to centimeters
                mf.guidanceLineDistanceOff = (short)Math.Round(distanceFromCurrentLinePivot * 1000.0, MidpointRounding.AwayFromZero);
                mf.guidanceLineSteerAngle = (short)(steerAngle * 100);
            }
            else
            {
                mf.guidanceLineDistanceOff = 32000;
                ResetUturn = true;
            }
            if (ResetUturn) mf.yt.ResetYouTurn();
        }
    }
}