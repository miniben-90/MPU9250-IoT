using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace MPU9250
{
    class KalmanFilter
    {
        private double Q_angle = 0.001,
                       Q_bias = 0.003,
	                   R_measure = 0.03,
                       angle = 0,
                       bias = 0,
                       rate = 0,
                       S = 0,
                       Y = 0;

        private double[][] P = new double[2][] { new double[2] { 0.0, 0.0 }, new double[2] { 0.0, 0.0 } };

        private double[] K = new double[2] { 0.0, 0.0 };

        public KalmanFilter()
        {
            this.P[0][0] = 0.0;
            this.P[0][1] = 0.0;
            this.P[1][0] = 0.0;
            this.P[1][1] = 0.0;
        }

        public double getAngle(double newAngle, double newRate, long dt)
        {
            try
            {
                this.rate = newRate - this.bias;
                this.angle += dt * this.rate;

                this.P[0][0] += dt * (dt * this.P[1][1] - this.P[0][1] - this.P[1][0] + this.Q_angle);
                this.P[0][1] -= dt * this.P[1][1];
                this.P[1][0] -= dt * this.P[1][1];
                this.P[1][1] += this.Q_bias * dt;

                this.S = this.P[0][0] + this.R_measure;

                this.K[0] = this.P[0][0] / this.S;
                this.K[1] = this.P[1][0] / this.S;

                this.Y = newAngle - this.angle;

                this.angle += this.K[0] * this.Y;
                this.bias += this.K[1] * this.Y;

                this.P[0][0] -= this.K[0] * this.P[0][0];
                this.P[0][1] -= this.K[0] * this.P[0][1];
                this.P[1][0] -= this.K[1] * this.P[0][0];
                this.P[1][1] -= this.K[1] * this.P[0][1];
            } catch(Exception err)
            {
                Debug.WriteLine(err.Message);
            }
            return this.angle;
        }
        
        public double getRate() { return this.rate; }
        public double getQAngle() { return this.Q_angle; }
        public double getQbias() { return this.Q_bias; }
        public double getRmeasure() { return this.R_measure; }
        
	    public void setAngle(double value) { this.angle = value; }
        public void setQangle(double value) { this.Q_angle = value; }
        public void setQbias(double value) { this.Q_bias = value; }
        public void setRmeasure(double value) { this.angle = value; }
    }
}
