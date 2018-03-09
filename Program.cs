using System;

namespace Robotics
{
    class MainClass
    {
        public static void Main(string[] args)
        {



            double[] inputs = new double[12];
            
            // Generates a random input to test the other functions.
            Random rand = new Random();
            Random rand2 = new Random();
            for (int i = 0; i < 12; i++){
                double number = rand.NextDouble();
                double number2 = rand2.Next(0,5);
                double sum = number + number2;
                if((sum) < 3){
                    inputs[i] = sum;
                }
                Console.WriteLine(i + " " + inputs[i]);
            }
            
            
            
            Cleaner vacuum = new Cleaner();
            vacuum.Run(inputs);
            
            
                    
        }

        public class Cleaner
        {
            double vLeft = 3;
            double vRight =1;
            double[,,] weights = new double[200,20, 8];
            double[] hiddenlayerSum = new double[6];
            double[] fitness = new double[200];
            int k = 3;
            double crossoverProb = 0.01;
            double mutationProb = 0.03;
            Random rand = new Random();
            double[] positionAndAngle = new double[3];

            double x;
            double y;
            double r;
            double l;
            double iccx;
            double iccy;
            double angle;
            double omega;
            double velocity;
            
            
            public void Run(double[] inputs){
                MakeWeights();
                //NeuralNetwork(0,inputs);

                x = 1;
                y = 1;
                angle = 37;
                
                // Printing the outputs to see if there are changes -> there are. Calculations seem to be working
                for (int i = 0; i < 10; i++)
                {
                    NeuralNetwork(0, inputs);
                    NextMove();
                    Console.WriteLine();
                }
                //GA();
            }
            
            // Generates weights for the 200 robots
            public void MakeWeights(){
                double number;
                for (int index = 0; index < 200; index++){
                    for (int i = 0; i < 20; i++){
                        if (i < 14)
                        {
                            for (int j = 0; j < 8; j++)
                            {
                                number = rand.NextDouble();
                                if(number > 0.5){
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
                                if(number > 0.4){
                                    number = 0; // Weights can be adjusted depending on how it effects the robot, the ideal size of the weights is not yet known.
                                }
                                weights[index, i, j] = number;
                            }
                        }
                    }
                }
            }
            
            
            // Neural network with hidden layer consisting of 6 nodes
            public void NeuralNetwork(int robotNr,double[] inputs){
                hiddenlayerSum = new double[6];
                for (int i = 0; i < 14; i++){
                    for (int j = 0; j < 6; j++){
                        if(i == 12){
                            hiddenlayerSum[j] += (vLeft * weights[robotNr,i, j]);
                            continue;
                        }
                        if(i == 13)
                        {
                            hiddenlayerSum[j] += (vRight * weights[robotNr, i, j]);
                            continue;
                        }
                        hiddenlayerSum[j] += (inputs[i] * weights[robotNr, i, j]);
                    }
                }
                int index=0;
                vLeft = 0;
                vRight = 0;
                for (int i = 14; i < 20; i++){
                    for (int j = 0; j < 2; j++){
                        if(j == 0){
                            vLeft += (hiddenlayerSum[index] * weights[robotNr,i,j]);
                        }
                        else{
                            vRight += (hiddenlayerSum[index] * weights[robotNr,i,j]);
                        }
                    }
                    index++;
                }
                Console.WriteLine(vLeft +" " + vRight);
            }
            
            public void GA(){
                /*
                Step 1: Create a selection procedure --> Tournament selection
                    Sort the population --> array of the positions of the population --> position 1 in array = position of the robot with best fitness in population array.
                    This list can be used to remove robots who do not perform well --> replace with newly created robot.
                Step 2: Create mutation, crossover (one point and uniform) --> these happen with a certain probability.                
                */
                
                
                for (int i = 0; i < 200; i++)
                {
                    fitness[i] = rand.Next(1, 101);
                    //Console.WriteLine(i + " " + fitness[i]);
                }


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
                    
                    for (int i = 0; i < 200;i++){
                        double number = rand.NextDouble();
                        if(number < crossoverProb){
                            robotA = rand.Next(0,200);
                            robotB = rand.Next(0,200);
                            Crossover(robotA,robotB);
                        }
                    }
                    
                    for (int i = 0; i < 200;i++){
                        double number = rand.NextDouble();
                        if(number < mutationProb){
                            robotA = rand.Next(0,200);
                            Mutation(robotA);
                        }
                    }
                }
            }
            
            // One point crossover
            public void Crossover(int robotA, int robotB){
                int position = rand.Next(1,20);
                int count = 0;
                double temp;
                for (int i = position; i < 20; i++){
                    for (int j = 0; j < 8; j++){
                        temp = weights[robotA, i, j];
                        weights[robotA, i, j] = weights[robotB, i, j];
                        weights[robotB, i, j] = temp;
                        count++;
                    }
                }
            }
            
            // Uniform Crossover
            public void UniformCross(int robotA, int robotB){
                int numberOfCross = rand.Next(1, 10);
                int position1;
                int position2;
                int position3;
                double temp;

                for (int i = 0; i < numberOfCross;i++){
                    position1 = rand.Next(0,20);
                    position2 = rand.Next(0,8);
                    position3 = rand.Next(0,2);
                    
                    if(position1 < 14){
                        temp = weights[robotA, position1, position2];
                        weights[robotA, position1, position2] = weights[robotB, position1, position2];
                        weights[robotB, position1, position2] = temp;
                    }
                    else{
                        temp = weights[robotA, position1, position3];
                        weights[robotA, position1, position3] = weights[robotB, position1, position3];
                        weights[robotB, position1, position3] = temp;
                    }
                }
                Console.WriteLine();

            }
  
            // Mututation
            public void Mutation(int robotA) { 
                int position1 = rand.Next(0,20);
                int position2 = rand.Next(0,8);
                int position3 = rand.Next(0,2);
                double value = rand.NextDouble();
                if(value > 0.5){
                    value -= 0.4;
                }
                if(position1 > 13){
                    weights[robotA, position1, position3] = value;
                }
                else{
                    weights[robotA, position1, position2] = value;
                }
            }
            
            

            public void NextMove()
            {

                l = 3;
                //x =  1;
                //y = 2;
                //angle = 0;
                

                R();
                Omega();
                ICCx();
                ICCy();
                Speed();
                RotationMatrix();
                
                
                

            }
            
            public void Speed(){
                velocity = (vLeft + vRight) / 2;
            }
            
            public void ICCx(){
                iccx = x - r * Math.Sin(angle);
                Console.WriteLine(iccx + " iccx");
            }
            public void ICCy(){
                iccy = y + r * Math.Cos(angle);
                Console.WriteLine(iccy + " iccy");
            }
            /*
            public double VRight(double omega, double R, double l){
                vRight = omega * (R + l / 2);
                return vRight;
            }
            public double Vleft(double omega, double R, double l){
                vLeft = omega * (R - l / 2);
                return vLeft;
            }
            */
            public void R(){
                
                r = (l / 2) * ((vLeft + vRight) / (vRight - vLeft));
                Console.WriteLine(r + " r");
            }
            public void Omega(){
                omega = (vRight - vLeft) / l;
                Console.WriteLine(omega + " omega");
            }
            
            public void RotationMatrix(){
                positionAndAngle = new double[3];
                double[,] matrix1 = new double[3,3];
                matrix1[0, 0] = Math.Cos(omega);
                matrix1[0, 1] = -Math.Sin(omega);
                matrix1[1, 0] = Math.Sin(omega);
                matrix1[1, 1] = Math.Cos(omega);
                matrix1[2, 2] = 1;

                double[] matrix2 =new double[3];
                matrix2[0] = x - iccx;
                matrix2[1] = y - iccy;
                matrix2[2] = angle;

                double[] matrix3 = new double[3];
                matrix3[0] = iccx;
                matrix3[1] = iccy;
                matrix3[2] = omega;

                for (int i = 0; i < 3; i++){
                    for (int j = 0; j < 3;j++){
                        positionAndAngle[i] += matrix1[i, j] * matrix2[j];
                    }
                }
                for (int i = 0; i < 3; i++){
                    positionAndAngle[i] += matrix3[i];
                    Console.WriteLine(positionAndAngle[i]);
                }

                x = positionAndAngle[0];
                y = positionAndAngle[1];
                angle = positionAndAngle[2];
            }
        }
    }
}
