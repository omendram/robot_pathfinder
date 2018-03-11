using System;
using System.Collections.Generic;
using System.ComponentModel;
using System.Data;
using System.Drawing;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows.Forms;

namespace RotatingPlane
{
    public partial class RotatingPlane : Form
    {
        Bitmap bmp = new Bitmap(1500, 1500);
        Graphics g;
        double angle = 0;
        int pointerAngle = 0;
        int _x = 110;
        int _y = 110;
        double VL = 2;
        double VR = 2;
        public int[,] availableArea = new int[6, 10];
        public int stepsTaken = 0;
        public int collisions = 0;
        public int cleanedArea = 0;
        double[] sensorInputs = new double[12];
        public double[] fitness = new double[200];
        double[,,] weights = new double[200, 20, 8];
        double[] hiddenlayerSum = new double[6];
        Random rand = new Random();
        double crossoverProb = 0.03;
        double mutationProb = 0.05;
        int k = 3;

        public RotatingPlane()
        {
            InitializeComponent();


            MakeWeights();
        }

        // Generates weights for the 200 robots
        public void MakeWeights()
        {
            double number;
            

            for (int index = 0; index < 200; index++)
            {
                for (int i = 0; i < 20; i++)
                {
                    if (i < 14)
                    {
                        for (int j = 0; j < 8; j++)
                        {
                            number = rand.NextDouble();
                            if (number > 0.5)
                            {
                                //number -= 0.4; // Weights can be adjusted depending on how it effects the robot, the ideal size of the weights is not yet known.
                            }
                            weights[index, i, j] = number;
                        }
                    }
                    if (i >= 14)
                    {
                        for (int j = 0; j < 2; j++)
                        {
                            number = rand.NextDouble();
                            if (number > 0.4)
                            {
                                number = 0; // Weights can be adjusted depending on how it effects the robot, the ideal size of the weights is not yet known.
                            }
                            weights[index, i, j] = number;
                        }
                    }
                }
            }
        }

        // Neural network with hidden layer consisting of 6 nodes
        public void NeuralNetwork(int robotNr)
        {
            hiddenlayerSum = new double[6];
            for (int i = 0; i < 14; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    if (i == 12)
                    {
                        hiddenlayerSum[j] += (VL * weights[robotNr, i, j]);
                        continue;
                    }
                    if (i == 13)
                    {
                        hiddenlayerSum[j] += (VR * weights[robotNr, i, j]);
                        continue;
                    }
                    hiddenlayerSum[j] += (sensorInputs[i] * weights[robotNr, i, j]);
                }
            }
            int index = 0;
            VL = 0;
            VR = 0;
            for (int i = 14; i < 20; i++)
            {
                for (int j = 0; j < 2; j++)
                {
                    if (j == 0)
                    {
                        VL += (hiddenlayerSum[index] * weights[robotNr, i, j]);
                    }
                    else
                    {
                        VR += (hiddenlayerSum[index] * weights[robotNr, i, j]);
                    }
                }
                index++;
            }
            Console.WriteLine(VL + " VL VR " + VR);
        }

        public void GA()
        {
            /*
            Step 1: Create a selection procedure --> Tournament selection
                Sort the population --> array of the positions of the population --> position 1 in array = position of the robot with best fitness in population array.
                This list can be used to remove robots who do not perform well --> replace with newly created robot.
            Step 2: Create mutation, crossover (one point and uniform) --> these happen with a certain probability.                
            */
            
            for (int dab = 0; dab < 1; dab++)
            {

                int bestOutOfTournament = 0;
                int index = 0;
                double bestFitness = 0;
                int[] winners = new int[50];

                //Complete tournament 20 times for k individuals and collect winners in array, can be adjusted to more or less ---------------------------------------------------
                for (int j = 0; j < 20; j++)
                {
                    for (int i = 0; i < k; i++)
                    {
                        index = rand.Next(0, 200);
                        if (bestFitness < fitness[index])
                        {
                            bestFitness = fitness[index];
                            bestOutOfTournament = index;
                        }
                    }
                    winners[j] = bestOutOfTournament;
                }

                for (int j = 0; j < 20; j++)
                {
                    // Find index of worst individual ------------------------------------------------------------------------------------------
                    int worstfitnessindex = 0;
                    double min = double.MaxValue;
                    for (int i = 0; i < 200; i++)
                    {
                        if (fitness[i] < min)
                        {
                            min = fitness[i];
                            worstfitnessindex = i;
                        }
                    }
                    // Replace the worst performing individual with an offspring of the winner of the tournament. --------------------------------
                    for (int a = 0; a < 20; a++)
                    {
                        for (int b = 0; b < 8; b++)
                        {
                            weights[worstfitnessindex, a, b] = weights[winners[j], a, b];
                        }
                    }
                }

                // Crossover between random pair with probability p. -------------------------------------------------------------------------------
                int robotA;
                int robotB;

                for (int i = 0; i < 200; i++)
                {
                    double number = rand.NextDouble();
                    if (number < crossoverProb)
                    {
                        robotA = rand.Next(0, 200);
                        robotB = rand.Next(0, 200);
                        Crossover(robotA, robotB);
                    }
                }

                for (int i = 0; i < 200; i++)
                {
                    double number = rand.NextDouble();
                    if (number < mutationProb)
                    {
                        robotA = rand.Next(0, 200);
                        Mutation(robotA);
                    }
                }
            }
        }

        // One point crossover
        public void Crossover(int robotA, int robotB)
        {
            int position = rand.Next(1, 20);
            int count = 0;
            double temp;
            for (int i = position; i < 20; i++)
            {
                for (int j = 0; j < 8; j++)
                {
                    temp = weights[robotA, i, j];
                    weights[robotA, i, j] = weights[robotB, i, j];
                    weights[robotB, i, j] = temp;
                    count++;
                }
            }
        }

        // Uniform Crossover
        public void UniformCross(int robotA, int robotB)
        {
            int numberOfCross = rand.Next(1, 10);
            int position1;
            int position2;
            int position3;
            double temp;

            for (int i = 0; i < numberOfCross; i++)
            {
                position1 = rand.Next(0, 20);
                position2 = rand.Next(0, 8);
                position3 = rand.Next(0, 2);

                if (position1 < 14)
                {
                    temp = weights[robotA, position1, position2];
                    weights[robotA, position1, position2] = weights[robotB, position1, position2];
                    weights[robotB, position1, position2] = temp;
                }
                else
                {
                    temp = weights[robotA, position1, position3];
                    weights[robotA, position1, position3] = weights[robotB, position1, position3];
                    weights[robotB, position1, position3] = temp;
                }
            }
            Console.WriteLine();

        }

        // Mututation
        public void Mutation(int robotA)
        {
            int position1 = rand.Next(0, 20);
            int position2 = rand.Next(0, 8);
            int position3 = rand.Next(0, 2);
            double value = rand.NextDouble();
            if (value > 0.5)
            {
                value -= 0.4;
            }
            if (position1 > 13)
            {
                weights[robotA, position1, position3] = value;
            }
            else
            {
                weights[robotA, position1, position2] = value;
            }
        }

        private void RotatingPlane_Load(object sender, EventArgs e)
        {
            g = Graphics.FromImage(bmp);
            pictureBox1.Image = bmp;
            RotationTimer.Start();
        }

        private int[] LineCoord(int angleIn, int radius, int center)
        {
            angleIn = angleIn + 90;
            int[] coord = new int[2]; // Setting up the int array for return
            angleIn %= 360;
            angleIn *= 1;

            if (angleIn >= 0 && angleIn <= 180)
            {
                coord[0] = center + (int)(radius * Math.Sin(Math.PI * angleIn / 180));
                coord[1] = center - (int)(radius * Math.Cos(Math.PI * angleIn / 180));
            }
            else
            {
                coord[0] = center - (int)(radius * -Math.Sin(Math.PI * angleIn / 180));
                coord[1] = center - (int)(radius * Math.Cos(Math.PI * angleIn / 180));
            }
            return coord;
        }

        private void RotationTimer_Tick(object sender, EventArgs e)
        {
            g = Graphics.FromImage(bmp);
            g.Clear(Color.White);
            pictureBox1.Image = bmp;
            g.FillEllipse(Brushes.Pink, _x-30, _y-30, 60, 60);
        
            Point point1 = new Point(LineCoord(pointerAngle, 30, _x)[0], LineCoord(pointerAngle, 30, _y)[1]);
            Point point2 = new Point(_x, _y);
            g.DrawLine(Pens.Black, point2, point1);
             
            g.DrawRectangle(Pens.Black, 30, 30, 600, 360);
            g.FillRectangle(Brushes.Black, 300, 200, 100, 50);

            // USE VL and VR to Calculate Omega, Angle of Turn
            // MOTION RELATED MODEL OF ROBOT

            float nextX;
            float nextY;

            if (VL != VR)
            {
                double l = 60;
                double _Omega = (VR - VL) / l;
                
                double _Radius = (l / 2) * ((VL + VR) / (VR - VL));
                double _ICCx = _x - _Radius * Math.Sin(angle);
                double _ICCy = _y + _Radius * Math.Cos(angle);
                angle = _Omega + angle;
                nextX = (float)(_ICCx + Math.Cos(_Omega) * (_x - _ICCx) - Math.Sin(_Omega) * (_y - _ICCy));
                nextY = (float)(_ICCy + Math.Sin(_Omega) * (_x - _ICCx) + Math.Cos(_Omega) * (_y - _ICCy));
            } else {
                nextY = (int)(_y + (float)(Math.Ceiling(Math.Sin((float)angle)) * VL));
                nextX = (int)(_x + (float)(Math.Ceiling(Math.Cos((float)angle)) * VR));
            }

            if (nextX != _x || nextY != _y)
            {

                // POINTER 
                pointerAngle = (int)angle + (int)(Math.Atan((nextY - _y) / (nextX - _x)) * 180 / Math.PI);

                _x = (int)nextX;
                _y = (int)nextY;
                 
                if (_x < 30 || _y < 30 || _x > 630 || _y > 390)
                {
                    collisions++;
                } else
                {
                    var factorX = (int)Math.Floor((double)_x / 60);
                    var factorY = (int)Math.Floor((double)_y / 60);

                    if (availableArea[factorX, factorY] != 1)
                    {
                        availableArea[factorX, factorY] = 1;
                        cleanedArea++;
                    }
                }

                stepsTaken++;
            }

            // END OF VL and  VR STUFF

            if (pointerAngle == 720)
            {
                pointerAngle = 0;
            }
      
            g.Dispose();
        }
        
    }
}
