using OpenTK.Graphics.OpenGL;
using System;
using System.Collections.Generic;

namespace AgOpenGPS
{
    public class CABLine
    {
        public double abHeading, abLength;
        public double angVel;

        public bool isABValid;

        //the current AB guidance line
        public List<vec3> curlist = new List<vec3>
            {
                new vec3(0.0,0.0,0.0),
                new vec3(0.0,0.0,0.0)
            };

        //List of all available ABLines
        public List<CABLines> lineArr = new List<CABLines>();

        public int numABLines, numABLineSelected;

        public double howManyPathsAway, moveDistance;
        public bool isABLineBeingSet;
        public bool isABLineSet, isABLineLoaded;
        public bool isHeadingSameWay = true;
        public bool isBtnABLineOn;

        //the reference line endpoints
        public vec2 refABLineP1 = new vec2(0.0, 0.0);
        public vec2 refABLineP2 = new vec2(0.0, 1.0);

        //the two inital A and B points
        public vec2 refPoint1 = new vec2(0.2, 0.15);
        public vec2 refPoint2 = new vec2(0.3, 0.3);

        public double snapDistance, lastSecond = 0;
        public int lineWidth;

        //design
        public vec2 desPoint1 = new vec2(0.2, 0.15);
        public vec2 desPoint2 = new vec2(0.3, 0.3);
        public double desHeading = 0;
        public vec2 desP1 = new vec2(0.0, 0.0);
        public vec2 desP2 = new vec2(999997, 1.0);
        public string desName = "";

        //pointers to mainform controls
        private readonly FormGPS mf;

        public CABLine(FormGPS _f)
        {
            //constructor
            mf = _f;
            //isOnTramLine = true;
            lineWidth = Properties.Settings.Default.setDisplay_lineWidth;
            abLength = Properties.Settings.Default.setAB_lineLength;
        }

        public void BuildCurrentABLineList(vec3 pivot, vec3 steer)
        {
            double dx, dy;

            lastSecond = mf.secondsSinceStart;

            //move the ABLine over based on the overlap amount set in
            double widthMinusOverlap = mf.tool.toolWidth - mf.tool.toolOverlap;

            //x2-x1
            dx = refABLineP2.easting - refABLineP1.easting;
            //z2-z1
            dy = refABLineP2.northing - refABLineP1.northing;

            double distanceFromRefLine = ((dy * mf.guidanceLookPos.easting) - (dx * mf.guidanceLookPos.northing) + (refABLineP2.easting
                                    * refABLineP1.northing) - (refABLineP2.northing * refABLineP1.easting))
                                        / Math.Sqrt((dy * dy) + (dx * dx));

            mf.gyd.isLateralTriggered = false;

            isHeadingSameWay = Math.PI - Math.Abs(Math.Abs(pivot.heading - abHeading) - Math.PI) < glm.PIBy2;

            if (mf.yt.isYouTurnTriggered) isHeadingSameWay = !isHeadingSameWay;

            //Which ABLine is the vehicle on, negative is left and positive is right side
            double RefDist = (distanceFromRefLine + (isHeadingSameWay ? mf.tool.toolOffset : -mf.tool.toolOffset)) / widthMinusOverlap;
            if (RefDist < 0) howManyPathsAway = (int)(RefDist - 0.5);
            else howManyPathsAway = (int)(RefDist + 0.5);

            //depending which way you are going, the offset can be either side
            vec2 point1 = new vec2((Math.Cos(-abHeading) * (widthMinusOverlap * howManyPathsAway + (isHeadingSameWay ? -mf.tool.toolOffset : mf.tool.toolOffset))) + refPoint1.easting,
            (Math.Sin(-abHeading) * ((widthMinusOverlap * howManyPathsAway) + (isHeadingSameWay ? -mf.tool.toolOffset : mf.tool.toolOffset))) + refPoint1.northing);

            //create the new line extent points for current ABLine based on original heading of AB line
            curlist.Clear();
            curlist.Add(new vec3(point1.easting - (Math.Sin(abHeading) * abLength), point1.northing - (Math.Cos(abHeading) * abLength), abHeading));
            curlist.Add(new vec3(point1.easting + (Math.Sin(abHeading) * abLength), point1.northing + (Math.Cos(abHeading) * abLength), abHeading));

            isABValid = true;
        }

        public void GetCurrentABLine(vec3 pivot, vec3 steer)
        {
            //build new current ref line if required
            if (!isABValid || ((mf.secondsSinceStart - lastSecond) > 0.66 && (!mf.isAutoSteerBtnOn || mf.mc.steerSwitchValue != 0)))
                BuildCurrentABLineList(pivot, steer);

            mf.gyd.CalculateSteerAngle(pivot, steer, ref curlist, isHeadingSameWay, mf.isStanleyUsed);
        }

        public void DrawABLines()
        {
            //Draw AB Points
            GL.PointSize(8.0f);
            GL.Begin(PrimitiveType.Points);

            GL.Color3(0.95f, 0.0f, 0.0f);
            GL.Vertex3(refPoint1.easting, refPoint1.northing, 0.0);
            GL.Color3(0.0f, 0.90f, 0.95f);
            GL.Vertex3(refPoint2.easting, refPoint2.northing, 0.0);
            GL.End();

            if (mf.font.isFontOn && !isABLineBeingSet)
            {
                mf.font.DrawText3D(refPoint1.easting, refPoint1.northing, "&A");
                mf.font.DrawText3D(refPoint2.easting, refPoint2.northing, "&B");
            }

            GL.PointSize(1.0f);

            //Draw reference AB line
            GL.LineWidth(lineWidth);
            GL.Enable(EnableCap.LineStipple);
            GL.LineStipple(1, 0x0F00);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(0.930f, 0.2f, 0.2f);
            GL.Vertex3(refABLineP1.easting, refABLineP1.northing, 0);
            GL.Vertex3(refABLineP2.easting, refABLineP2.northing, 0);
            GL.End();
            GL.Disable(EnableCap.LineStipple);

            //draw current AB Line
            GL.LineWidth(lineWidth);
            GL.Begin(PrimitiveType.Lines);
            GL.Color3(0.95f, 0.20f, 0.950f);
            GL.Vertex3(curlist[0].easting, curlist[0].northing, 0.0);
            GL.Vertex3(curlist[1].easting, curlist[1].northing, 0.0);
            GL.End();

            //ABLine currently being designed
            if (isABLineBeingSet)
            {
                GL.LineWidth(lineWidth);
                GL.Begin(PrimitiveType.Lines);
                GL.Color3(0.95f, 0.20f, 0.950f);
                GL.Vertex3(desP1.easting, desP1.northing, 0.0);
                GL.Vertex3(desP2.easting, desP2.northing, 0.0);
                GL.End();

                GL.Color3(0.2f, 0.950f, 0.20f);
                mf.font.DrawText3D(desPoint1.easting, desPoint1.northing, "&A");
                mf.font.DrawText3D(desPoint2.easting, desPoint2.northing, "&B");
            }

            if (mf.isSideGuideLines && mf.camera.camSetDistance > mf.tool.toolWidth * -120)
            {
                //get the tool offset and width
                double toolOffset = mf.tool.toolOffset * 2;
                double toolWidth = mf.tool.toolWidth - mf.tool.toolOverlap;
                double cosHeading = Math.Cos(-abHeading);
                double sinHeading = Math.Sin(-abHeading);

                GL.Color3(0.756f, 0.7650f, 0.7650f);
                GL.Enable(EnableCap.LineStipple);
                GL.LineStipple(1, 0x0303);

                GL.LineWidth(lineWidth);
                GL.Begin(PrimitiveType.Lines);

                /*
                for (double i = -2.5; i < 3; i++)
                {
                    GL.Vertex3((cosHeading * ((mf.tool.toolWidth - mf.tool.toolOverlap) * (howManyPathsAway + i))) + refPoint1.easting, (sinHeading * ((mf.tool.toolWidth - mf.tool.toolOverlap) * (howManyPathsAway + i))) + refPoint1.northing, 0);
                    GL.Vertex3((cosHeading * ((mf.tool.toolWidth - mf.tool.toolOverlap) * (howManyPathsAway + i))) + refPoint2.easting, (sinHeading * ((mf.tool.toolWidth - mf.tool.toolOverlap) * (howManyPathsAway + i))) + refPoint2.northing, 0);
                }
                */

                if (isHeadingSameWay)
                {
                    GL.Vertex3((cosHeading * (toolWidth + toolOffset)) + curlist[0].easting, (sinHeading * (toolWidth + toolOffset)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (toolWidth + toolOffset)) + curlist[1].easting, (sinHeading * (toolWidth + toolOffset)) + curlist[1].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth + toolOffset)) + curlist[0].easting, (sinHeading * (-toolWidth + toolOffset)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth + toolOffset)) + curlist[1].easting, (sinHeading * (-toolWidth + toolOffset)) + curlist[1].northing, 0);

                    toolWidth *= 2;
                    GL.Vertex3((cosHeading * toolWidth) + curlist[0].easting, (sinHeading * toolWidth) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * toolWidth) + curlist[1].easting, (sinHeading * toolWidth) + curlist[1].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth)) + curlist[0].easting, (sinHeading * (-toolWidth)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth)) + curlist[1].easting, (sinHeading * (-toolWidth)) + curlist[1].northing, 0);
                }
                else
                {
                    GL.Vertex3((cosHeading * (toolWidth - toolOffset)) + curlist[0].easting, (sinHeading * (toolWidth - toolOffset)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (toolWidth - toolOffset)) + curlist[1].easting, (sinHeading * (toolWidth - toolOffset)) + curlist[1].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth - toolOffset)) + curlist[0].easting, (sinHeading * (-toolWidth - toolOffset)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth - toolOffset)) + curlist[1].easting, (sinHeading * (-toolWidth - toolOffset)) + curlist[1].northing, 0);

                    toolWidth *= 2;
                    GL.Vertex3((cosHeading * toolWidth) + curlist[0].easting, (sinHeading * toolWidth) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * toolWidth) + curlist[1].easting, (sinHeading * toolWidth) + curlist[1].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth)) + curlist[0].easting, (sinHeading * (-toolWidth)) + curlist[0].northing, 0);
                    GL.Vertex3((cosHeading * (-toolWidth)) + curlist[1].easting, (sinHeading * (-toolWidth)) + curlist[1].northing, 0);
                }

                GL.End();
                GL.Disable(EnableCap.LineStipple);
            }


            mf.yt.DrawYouTurn();

            GL.PointSize(1.0f);
            GL.LineWidth(1);
        }

        public void BuildTram()
        {
            mf.tram.BuildTramBnd();

            mf.tram.tramList?.Clear();
            mf.tram.tramArr?.Clear();
            List<vec2> tramRef = new List<vec2>();

            bool isBndExist = mf.bnd.bndArr.Count != 0;

            double pass = 0.5;
            double hsin = Math.Sin(abHeading);
            double hcos = Math.Cos(abHeading);

            //divide up the AB line into segments
            for (int i = 0; i < 3200; i += 4)
            {
                tramRef.Add(new vec2((hsin * i) + refABLineP1.easting, (hcos * i) + refABLineP1.northing));
            }

            //create list of list of points of triangle strip of AB Highlight
            double headingCalc = abHeading + glm.PIBy2;
            hsin = Math.Sin(headingCalc);
            hcos = Math.Cos(headingCalc);

            mf.tram.tramList?.Clear();
            mf.tram.tramArr?.Clear();

            //no boundary starts on first pass
            int cntr = 0;
            if (isBndExist) cntr = 1;

            for (int i = cntr; i < mf.tram.passes; i++)
            {
                mf.tram.tramArr = new List<vec2>();
                mf.tram.tramArr.Capacity = 128;

                mf.tram.tramList.Add(mf.tram.tramArr);

                double hSinWidth = hsin * ((mf.tram.tramWidth * (pass + i)) - mf.tram.halfWheelTrack + mf.tool.halfToolWidth);
                double hCosWidth = hcos * ((mf.tram.tramWidth * (pass + i)) - mf.tram.halfWheelTrack + mf.tool.halfToolWidth);

                for (int j = 0; j < tramRef.Count; j++)
                {
                    vec2 P1 = new vec2(tramRef[j].easting + hSinWidth, tramRef[j].northing + hCosWidth);

                    if (isBndExist)
                    {
                        if (mf.bnd.bndArr[0].IsPointInsideBoundaryEar(P1))
                        {
                            mf.tram.tramArr.Add(P1);
                        }
                    }
                    else
                    {
                        mf.tram.tramArr.Add(P1);
                    }
                }
            }

            for (int i = cntr; i < mf.tram.passes; i++)
            {
                mf.tram.tramArr = new List<vec2>();
                mf.tram.tramArr.Capacity = 128;

                mf.tram.tramList.Add(mf.tram.tramArr);

                double hSinWidth = hsin * ((mf.tram.tramWidth * (pass + i)) + mf.tram.halfWheelTrack + mf.tool.halfToolWidth);
                double hCosWidth = hcos * ((mf.tram.tramWidth * (pass + i)) + mf.tram.halfWheelTrack + mf.tool.halfToolWidth);

                for (int j = 0; j < tramRef.Count; j++)
                {
                    vec2 P1 = new vec2(tramRef[j].easting + hSinWidth, tramRef[j].northing + hCosWidth);

                    if (isBndExist)
                    {
                        if (mf.bnd.bndArr[0].IsPointInsideBoundaryEar(P1))
                        {
                            mf.tram.tramArr.Add(P1);
                        }
                    }
                    else
                    {
                        mf.tram.tramArr.Add(P1);
                    }
                }
            }

            tramRef?.Clear();
            //outside tram

            if (mf.bnd.bndArr.Count == 0 || mf.tram.passes != 0)
            {
                //return;
            }
        }

        public void DeleteAB()
        {
            refPoint1 = new vec2(0.0, 0.0);
            refPoint2 = new vec2(0.0, 1.0);

            refABLineP1 = new vec2(0.0, 0.0);
            refABLineP2 = new vec2(0.0, 1.0);

            curlist[0] = new vec3(0.0, 0.0, 0.0);
            curlist[1] = new vec3(0.0, 1.0, 0.0);

            abHeading = 0.0;
            howManyPathsAway = 0.0;
            isABLineSet = false;
            isABLineLoaded = false;
        }

        public void SetABLineByHeading()
        {
            //heading is set in the AB Form
            refABLineP1.easting = refPoint1.easting - (Math.Sin(abHeading) * abLength);
            refABLineP1.northing = refPoint1.northing - (Math.Cos(abHeading) * abLength);

            refABLineP2.easting = refPoint1.easting + (Math.Sin(abHeading) * abLength);
            refABLineP2.northing = refPoint1.northing + (Math.Cos(abHeading) * abLength);

            refPoint2.easting = refABLineP2.easting;
            refPoint2.northing = refABLineP2.northing;

            isABLineSet = true;
            isABLineLoaded = true;
        }

        public void MoveABLine(double dist)
        {
            moveDistance += isHeadingSameWay ? dist : -dist;

            //calculate the new points for the reference line and points
            refPoint1.easting += Math.Cos(abHeading) * (isHeadingSameWay ? dist : -dist);
            refPoint1.northing -= Math.Sin(abHeading) * (isHeadingSameWay ? dist : -dist);

            refABLineP1.easting = refPoint1.easting - (Math.Sin(abHeading) * abLength);
            refABLineP1.northing = refPoint1.northing - (Math.Cos(abHeading) * abLength);

            refABLineP2.easting = refPoint1.easting + (Math.Sin(abHeading) * abLength);
            refABLineP2.northing = refPoint1.northing + (Math.Cos(abHeading) * abLength);

            refPoint2.easting = refABLineP2.easting;
            refPoint2.northing = refABLineP2.northing;

            isABValid = false;
        }
    }

    public class CABLines
    {
        public vec2 origin = new vec2(0.0,0.0);
        public double heading = 0;
        public string Name = "aa";
    }
}