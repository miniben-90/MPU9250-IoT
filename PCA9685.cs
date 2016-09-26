using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace PCA9685
{
    class PCA9685
    {
        private static byte MODE1 = 0x00
                            , MODE2 = 0x01
                            , SUBADR1 = 0x02
                            , SUBADR2 = 0x03
                            , SUBADR3 = 0x04
                            , ALLCALLADR = 0x05
                            , PRE_SCALE = 0xFE
                            , TestMode = 0xFF;


        private static float PRESCAL_MHZ = 25000000.0f;
        private static float PRESCAL_BIT = 4096.0f;

        private float FREQ = 60.0f;

        private I2cDevice device;
        private Boolean deviceReady = false;
        private bool debug = false;
        private double correctionFactor = 1.0;

        private static byte DEVICE_ADDRESS = 0x40;

        private static byte[][] ALL_LED_TAB = {
            new byte[] { 0xFA, 0xFB },
            new byte[] { 0xFC, 0xFD },
        };

        private static byte[][][] LED_TAB = {
            new byte[][] { new byte[] { 0x06, 0x07 }, new byte[] { 0x08, 0x09 } },
            new byte[][] { new byte[] { 0x0A, 0x0B }, new byte[] { 0x0C, 0x0D } },
            new byte[][] { new byte[] { 0x0E, 0x0F }, new byte[] { 0x10, 0x11 } },
            new byte[][] { new byte[] { 0x12, 0x13 }, new byte[] { 0x14, 0x15 } },
            new byte[][] { new byte[] { 0x16, 0x17 }, new byte[] { 0x18, 0x19 } },
            new byte[][] { new byte[] { 0x1A, 0x1B }, new byte[] { 0x1C, 0x1D } },
            new byte[][] { new byte[] { 0x1E, 0x1F }, new byte[] { 0x20, 0x21 } },
            new byte[][] { new byte[] { 0x22, 0x23 }, new byte[] { 0x24, 0x25 } },
            new byte[][] { new byte[] { 0x26, 0x27 }, new byte[] { 0x28, 0x29 } },
            new byte[][] { new byte[] { 0x2A, 0x2B }, new byte[] { 0x2C, 0x2D } },
            new byte[][] { new byte[] { 0x2E, 0x2F }, new byte[] { 0x30, 0x31 } },
            new byte[][] { new byte[] { 0x32, 0x33 }, new byte[] { 0x34, 0x35 } },
            new byte[][] { new byte[] { 0x36, 0x37 }, new byte[] { 0x38, 0x39 } },
            new byte[][] { new byte[] { 0x3A, 0x3B }, new byte[] { 0x3C, 0x3D } },
            new byte[][] { new byte[] { 0x3E, 0x3F }, new byte[] { 0x40, 0x41 } },
            new byte[][] { new byte[] { 0x42, 0x43 }, new byte[] { 0x44, 0x45 } },
        };

        public PCA9685()
        {
            connection();
        }


        private async void connection()
        {
            try
            {

                var settings = new I2cConnectionSettings(DEVICE_ADDRESS);
                settings.BusSpeed = I2cBusSpeed.StandardMode;

                string aqs = I2cDevice.GetDeviceSelector();
                var dis = await DeviceInformation.FindAllAsync(aqs);
                this.device = await I2cDevice.FromIdAsync(dis[0].Id, settings);
                if (this.device == null)
                {
                    var errno = string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id) + "\n";
                    Debug.WriteLine(errno);
                    return;
                }
                this.reset();
                this.setPWMFreq();
                this.deviceReady = true;
            }
            catch(Exception err)
            {
                Debug.WriteLine(err.Message);
            }
        }

        public double getValueInMs(double ms)
        {
            /**

            int val = 0;
		    float t1 = ((float) 1 / PCA9685_FREQ) * 1000.0f;
		    if (PCA9685_DEBUG) {
			    System.out.println("W[info]: t1 value : " + t1);
		    }
		    float t =(float) t1 / PCA9685_PRESCAL_BIT;
		    if (PCA9685_DEBUG) {
			    System.out.println("W[info]: t1 value : " + t);
		    }
		    float tmp = (float) (ms / t);
		    if (PCA9685_DEBUG) {
			    System.out.println("W[info]: t1 value : " + tmp);
		    }
		    tmp -= 1;
		    val = (int) Math.floor(tmp);
		    return val;

            **/

            float t1 = ((float) 1 / FREQ) * 1000.0f;
            float t = (float) t1 / PRESCAL_BIT;
            float tmp = (float)(ms / t);
            tmp -= 1;
            if (tmp < 0)
            {
                tmp = 0;
            }
            return (int) Math.Floor(tmp);
        }

        public void setPWM(int channel, int value)
        {
            if (this.device != null && this.deviceReady)
            {
                try
                {
                    if (channel < 0 && channel > 15)
                    {
                        throw new ArgumentException("illegal number");
                    }
                    if (value < 0 || value > 4095)
                    {
                        throw new ArgumentException("illegal number");
                    }

                    byte[] valBytes = BitConverter.GetBytes(value);

                    Debug.WriteLine(BitConverter.ToString(valBytes));
                    Array.Reverse(valBytes);
                    device.Write(new byte[] { LED_TAB[channel][0][0], valBytes[0] });
                    device.Write(new byte[] { LED_TAB[channel][0][1], valBytes[1] });
                    device.Write(new byte[] { LED_TAB[channel][1][0], valBytes[2] });
                    device.Write(new byte[] { LED_TAB[channel][1][1], valBytes[3] });

                }
                catch (Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }
        }
        
        public void setPWM(int channel, double on, double off)
        {
            if (this.device != null && this.deviceReady)
            {
                try
                {
                    if (channel < 0 && channel > 15)
                    {
                        throw new ArgumentException("illegal number");
                    }
                    if (on < 0 || on > 4096 || off < 0 || off > 4096)
                    {
                        throw new ArgumentException("illegal number");
                    }

                    byte[] valOn = new byte[2] {
                        (byte)((int) on & 0xFF),
                        (byte)((int) on >> 8),
                    };
                    byte[] valOff = new byte[2] {
                        (byte)((int) off & 0xFF),
                        (byte)((int) off >> 8),
                    };


                    byte[] valOntest = BitConverter.GetBytes(on);
                    byte[] valOfftest = BitConverter.GetBytes(off);

                    Debug.WriteLine(BitConverter.ToString(valOn));
                    Debug.WriteLine(BitConverter.ToString(valOff));

                    Debug.WriteLine(BitConverter.ToString(valOntest));
                    Debug.WriteLine(BitConverter.ToString(valOfftest));

                    device.Write(new byte[] { LED_TAB[channel][0][0] , valOn[0] });
                    device.Write(new byte[] { LED_TAB[channel][0][1] , valOn[1] });
                    device.Write(new byte[] { LED_TAB[channel][1][0] , valOff[0] });
                    device.Write(new byte[] { LED_TAB[channel][1][1] , valOff[1] });
                }
                catch (Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }
        }

        public void setAllPWM(double on, double off)
        {
            if (this.device != null && this.deviceReady)
            {
                try
                {
                    if (on < 0 || on > 4096 || off < 0 || off > 4096)
                    {
                        throw new ArgumentException("illegal number");
                    }

                    byte[] valOn = new byte[2] {
                        (byte)((int) on & 0xFF),
                        (byte)((int) on >> 8),
                    };
                    byte[] valOff = new byte[2] {
                        (byte)((int) off & 0xFF),
                        (byte)((int) off >> 8),
                    };

                    byte[] valOntest = BitConverter.GetBytes(on);
                    byte[] valOfftest = BitConverter.GetBytes(off);

                    Debug.WriteLine(BitConverter.ToString(valOn));
                    Debug.WriteLine(BitConverter.ToString(valOff));

                    Debug.WriteLine(BitConverter.ToString(valOntest));
                    Debug.WriteLine(BitConverter.ToString(valOfftest));
                    
                    device.Write(new byte[] { ALL_LED_TAB[0][0] , valOn[0] });
                    device.Write(new byte[] { ALL_LED_TAB[0][1] , valOn[1] });
                    device.Write(new byte[] { ALL_LED_TAB[1][0] , valOff[0] });
                    device.Write(new byte[] { ALL_LED_TAB[1][1] , valOff[1] });



                }
                catch (Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }
        }

        private void reset()
        {
            if (this.device != null)
            {
                try
                {
                    this.device.Write(new byte[] { MODE1, 0x00 });
                    Task.Delay(10).Wait();
                    this.device.Write(new byte[] { ALL_LED_TAB[1][1], 0x10 });
                } catch(Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }
        }

        private void setPWMFreq()
        {
            if (this.device != null)
            {
                try
                {
                    if (this.correctionFactor <= 0)
                        this.correctionFactor = 1.0;

                    double prescaleval = PRESCAL_MHZ;
                    prescaleval /= PRESCAL_BIT;
                    prescaleval /= FREQ;
                    prescaleval -= -1.0;

                    var prescale = Math.Floor(prescaleval * correctionFactor + 0.5);

                    byte[] oldmode = new byte[1];
                    this.device.WriteRead(new byte[] { MODE1 }, oldmode);
                    byte newmode = (byte)(oldmode[0] & 0x7F | 0x10);
                    this.device.Write(new byte[] { MODE1, newmode });
                    this.device.Write(new byte[] { PRE_SCALE, (byte) prescale });
                    this.device.Write(new byte[] { MODE1, oldmode[0] });
                    Task.Delay(1000).Wait();
                    this.device.Write(new byte[] { MODE1, 0x80 });
                }
                catch (Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }
        }
    }
}
