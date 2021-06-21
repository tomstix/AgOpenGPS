using System;
using System.Windows.Forms;

namespace AgOpenGPS
{
    public partial class FormEditAB : Form
    {
        private readonly FormGPS mf = null;

        private double snapAdj = 0;

        public FormEditAB(Form callingForm)
        {
            //get copy of the calling main form
            mf = callingForm as FormGPS;

            InitializeComponent();

            this.Text = gStr.gsEditABLine;
            nudMinTurnRadius.Controls[0].Enabled = false;
        }

        private void FormEditAB_Load(object sender, EventArgs e)
        {
            if (mf.isMetric)
            {
                nudMinTurnRadius.DecimalPlaces = 0;
                nudMinTurnRadius.Value = (int)((double)Properties.Settings.Default.setAS_snapDistance * mf.cm2CmOrIn);
            }
            else
            {
                nudMinTurnRadius.DecimalPlaces = 1;
                nudMinTurnRadius.Value = (decimal)Math.Round(((double)Properties.Settings.Default.setAS_snapDistance * mf.cm2CmOrIn), 1);
            }

            label1.Text = mf.unitsInCm;
            btnCancel.Focus();
            lblHalfSnapFtM.Text = mf.unitsFtM;
            lblHalfWidth.Text = (mf.tool.toolWidth * 0.5 * mf.m2FtOrM).ToString("N2");
            tboxHeading.Text = Math.Round(glm.toDegrees(mf.ABLine.refList[1].heading), 5).ToString();
        }

        private void tboxHeading_Enter(object sender, EventArgs e)
        {
            tboxHeading.Text = "";

            using (FormNumeric form = new FormNumeric(0, 360, Math.Round(glm.toDegrees(mf.ABLine.refList[1].heading), 5)))
            {
                DialogResult result = form.ShowDialog();
                if (result == DialogResult.OK)
                {
                    tboxHeading.Text = ((double)form.ReturnValue).ToString();
                    mf.ABLine.refList[1].heading = glm.toRadians((double)form.ReturnValue);
                    mf.ABLine.SetABLineByHeading();
                }
                else tboxHeading.Text = Math.Round(glm.toDegrees(mf.ABLine.refList[1].heading), 5).ToString();

            }

            mf.ABLine.isABValid = false;
            btnCancel.Focus();
        }

        private void nudMinTurnRadius_Enter(object sender, EventArgs e)
        {
            mf.KeypadToNUD((NumericUpDown)sender, this);
            btnCancel.Focus();
        }

        private void nudMinTurnRadius_ValueChanged(object sender, EventArgs e)
        {
            snapAdj = (double)nudMinTurnRadius.Value * mf.inOrCm2Cm * 0.01;
        }

        private void btnAdjRight_Click(object sender, EventArgs e)
        {
            mf.ABLine.MoveABLine(snapAdj);
        }

        private void btnAdjLeft_Click(object sender, EventArgs e)
        {
            mf.ABLine.MoveABLine(-snapAdj);
        }

        private void bntOk_Click(object sender, EventArgs e)
        {
            //index to last one. 
            int idx = mf.ABLine.numABLineSelected - 1;

            if (idx >= 0)
            {
                mf.ABLine.lineArr[idx].heading = mf.ABLine.refList[1].heading;
                //calculate the new points for the reference line and points
                mf.ABLine.lineArr[idx].origin.easting = mf.ABLine.refList[1].easting;
                mf.ABLine.lineArr[idx].origin.northing = mf.ABLine.refList[1].northing;
            }

            mf.FileSaveABLines();
            mf.ABLine.moveDistance = 0;

            mf.panelRight.Enabled = true;
            mf.ABLine.isABValid = false;
            Close();
        }

        private void btnCancel_Click(object sender, EventArgs e)
        {
            int last = mf.ABLine.numABLineSelected;
            mf.FileLoadABLines();

            mf.ABLine.numABLineSelected = last;
            mf.ABLine.refList[1] = new vec3(mf.ABLine.lineArr[last - 1].origin.easting, mf.ABLine.lineArr[last - 1].origin.northing, mf.ABLine.lineArr[last - 1].heading);
            mf.ABLine.SetABLineByHeading();
            mf.ABLine.isABLineSet = true;
            mf.ABLine.isABLineLoaded = true;
            mf.ABLine.moveDistance = 0;

            mf.panelRight.Enabled = true;
            mf.ABLine.isABValid = false;
            Close();
        }

        private void btnSwapAB_Click(object sender, EventArgs e)
        {
            mf.ABLine.SwapAB();
            tboxHeading.Text = Math.Round(glm.toDegrees(mf.ABLine.refList[1].heading), 5).ToString();
        }

        private void btnContourPriority_Click(object sender, EventArgs e)
        {
            if (mf.ABLine.isABLineSet)
            {
                mf.ABLine.MoveABLine(mf.gyd.distanceFromCurrentLinePivot);
            }
        }

        private void btnRightHalfWidth_Click(object sender, EventArgs e)
        {
            double dist = mf.tool.toolWidth;

            mf.ABLine.MoveABLine(dist * 0.5);
        }

        private void btnLeftHalfWidth_Click(object sender, EventArgs e)
        {
            double dist = mf.tool.toolWidth;

            mf.ABLine.MoveABLine(-dist * 0.5);
        }

        private void btnNoSave_Click(object sender, EventArgs e)
        {
            mf.ABLine.isABValid = false;
            Close();
        }

        private void cboxDegrees_SelectedIndexChanged(object sender, EventArgs e)
        {
            mf.ABLine.refList[1].heading = glm.toRadians(double.Parse(cboxDegrees.SelectedItem.ToString()));
            mf.ABLine.SetABLineByHeading();
            tboxHeading.Text = Math.Round(glm.toDegrees(mf.ABLine.refList[1].heading), 5).ToString();
        }
    }
}
