using System;
using System.Collections;
using System.Diagnostics;
using System.Threading.Tasks;
using Windows.Devices.Enumeration;
using Windows.Devices.I2c;

namespace MPU9250
{
    class AK8963
    {
        public delegate void akCallbackData(double[] data);

        private static byte WHIA_VALUE = 0x48,
        ADRESS = 0x0C,
        WIA = 0x00,
        INFO = 0x01,
        ST1 = 0x02,
        HXL = 0x03,
        HXH = 0x04,
        HYL = 0x05,
        HYH = 0x06,
        HZL = 0x07,
        HZH = 0x08,
        ST2 = 0x09,
        CNTL = 0x0A,
        RSV = 0x0B,
        ASTC = 0x0C,
        TS1 = 0x0D,
        TS2 = 0x0E,
        I2CDIS = 0x0F,
        ASAX = 0x10,
        ASAZ = 0x12,
        ASAY = 0x11;
        
        private static double mRes14bits = 10.0 * 4912.0 / 8190.0;
        private static double mRes16bits = 10.0 * 4912.0 / 32760.0;

        private double[] calibrationData = new double[3] { 0, 0, 0 };

        public static byte CNTL_MODE_OFF = 0x00, // Power-down mode
                           CNTL_MODE_SINGLE_MESURE = 0x01, // Single measurement mode
                           CNTL_MODE_CONTINUE_MESURE_1 = 0x02, // Continuous measurement mode 1
                           CNTL_MODE_CONTINUE_MESURE_2 = 0x06, // Continuous measurement mode 2
                           CNTL_MODE_EXT_TRIG_MESURE = 0x04, // External trigger measurement mode
                           CNTL_MODE_SELF_TEST_MODE = 0x08, // Self-test mode
                           CNTL_MODE_FULL_ROM_ACCESS = 0x0F;  // Fuse ROM access mode
        
        private I2cDevice mpu;
        private Boolean deviceReady = false;
        private bool debug = false;

        public AK8963()
        {
            this.debug = true;
            this.connection();
        }

        private async void connection()
        {
            try
            {
                var settings = new I2cConnectionSettings(ADRESS);
                settings.BusSpeed = I2cBusSpeed.FastMode;                       /* 400KHz bus speed */

                string aqs = I2cDevice.GetDeviceSelector();                     /* Get a selector string that will return all I2C controllers on the system */
                var dis = await DeviceInformation.FindAllAsync(aqs);            /* Find the I2C bus controller devices with our selector string             */
                this.mpu = await I2cDevice.FromIdAsync(dis[0].Id, settings);    /* Create an I2cDevice with our selected bus controller and I2C settings    */
                if (this.mpu == null)
                {
                    var errno = string.Format(
                        "Slave address {0} on I2C Controller {1} is currently in use by " +
                        "another application. Please ensure that no other applications are using I2C.",
                        settings.SlaveAddress,
                        dis[0].Id) + "\n";
                    Debug.WriteLine(errno);
                    return;
                }

                if (this.whoami() == WHIA_VALUE)
                {
                    if (this.debug) Debug.WriteLine("DEVICE IS OK {0}", this.whoami());
                    this.setCNTL(CNTL_MODE_CONTINUE_MESURE_1);
                    if (this.debug) Debug.WriteLine("SET CNTL TO CONTINUE MESURE 1");
                    this.setCalibrationData();
                    this.deviceReady = true;
                }
            }
            catch (Exception err)
            {
                Debug.WriteLine(err.Message);
            }
        }


        /**
         * Convert table of bits to byte
         **/
        public static byte ConvertToByte(BitArray bits)
        {
            if (bits.Length != 8)
            {
                throw new ArgumentException("illegal number of bits");
            }
            byte b = 0;
            if (bits.Get(0)) b++;
            if (bits.Get(1)) b += 2;
            if (bits.Get(2)) b += 4;
            if (bits.Get(3)) b += 8;
            if (bits.Get(4)) b += 16;
            if (bits.Get(5)) b += 32;
            if (bits.Get(6)) b += 64;
            if (bits.Get(7)) b += 128;
            return b;
        }

        /**
         * return if device is ready to work 
         **/
        public bool isReady()
        {
            return this.deviceReady;
        }

        public bool deviceIsReady()
        {
            bool val = false;

            if (this.mpu != null)
            {
                byte[] ReadBuf = new byte[1];
                this.mpu.WriteRead(new byte[] { ST1 }, ReadBuf);
                val = (ReadBuf[0] == 0x01);
            }
            return val; 
        }

        public void setCNTL(byte value)
        {
            if(this.mpu != null)
            {
                try
                {
                    this.mpu.Write(new byte[] { CNTL, value });
                    Task.Delay(10).Wait();
                } catch(Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
                
            }
        }

        public byte getCNTL()
        {
            byte data = 0x00;
            if (this.mpu != null)
            {
                byte[] ReadBuf = new byte[1];
                this.mpu.WriteRead(new byte[] { ST1 }, ReadBuf);
                data = ReadBuf[0];
            }
            return data;
        }

        /**
         * return device ID if it work it wll be = to 72dec (0x48hex)
         **/
        public short whoami()
        {
            short value = 0x00;

            if (this.mpu != null)
            {
                try
                {
                    byte[] ReadBuf = new byte[1];
                    this.mpu.WriteRead(new byte[] { WIA }, ReadBuf);
                    value = (short)ReadBuf[0];
                }
                catch (Exception err)
                {
                    Debug.WriteLine(err.Message);
                }
            }

            return value;
        }

        private void setCalibrationData()
        {
            //calibrationData

            if (this.mpu != null)
            {

                byte[] ReadBuf = new byte[3];
                this.mpu.WriteRead(new byte[] { ASAX }, ReadBuf);

                //calibrationData[0] = BitConverter.ToInt16(new byte[] { ReadBuf[0] }, 0);
                //calibrationData[1] = BitConverter.ToInt16(new byte[] { ReadBuf[1] }, 0);
                //calibrationData[2] = BitConverter.ToInt16(new byte[] { ReadBuf[2] }, 0);
                calibrationData[0] =  (ReadBuf[0] - 128) / 256 + 1;
                calibrationData[1] = (ReadBuf[1] - 128) / 256 + 1;
                calibrationData[2] = (ReadBuf[2] - 128) / 256 + 1;

                Debug.WriteLine(BitConverter.ToString(ReadBuf));

                Debug.WriteLine("CX={0},CY={1},CZ={2}", new object[] { calibrationData[0], calibrationData[1], calibrationData[2] });
            }


        }

        /**
         * return magnetometer data mx,my,mz
         **/
        public double[] getMagnetometer()
        {
            double[] data = new double[3] { 0, 0, 0 };

            if (this.mpu != null && this.deviceReady == true)
            {
                
                byte[] ReadBuf = new byte[7];
                this.mpu.WriteRead(new byte[] { HXL }, ReadBuf);
                if (ReadBuf[6] != 0x08)
                {
                    data[0] = BitConverter.ToInt16(new byte[] { ReadBuf[1], ReadBuf[0] }, 0) * mRes14bits * this.calibrationData[0];
                    data[1] = BitConverter.ToInt16(new byte[] { ReadBuf[3], ReadBuf[2] }, 0) * mRes14bits * this.calibrationData[1];
                    data[2] = BitConverter.ToInt16(new byte[] { ReadBuf[5], ReadBuf[4] }, 0) * mRes14bits * this.calibrationData[2];
                }
            }

            return data;
        }

        /**
         * return magnetometer data mx,my,mz
         **/
        public void getMagnetometer(akCallbackData callback)
        {
            this.getMagnetometer(callback, 1000);
        }

        /**
         * return magnetometer data mx,my,mz
         **/
        public void getMagnetometer(akCallbackData callback,int delay)
        {
            if (this.mpu != null && this.deviceReady == true)
            {
                for (;;)
                {
                    this.setCNTL(CNTL_MODE_CONTINUE_MESURE_1);

                    double[] data = new double[3] { 0, 0, 0 };
                    byte[] ReadBuf = new byte[7];
                    this.mpu.WriteRead(new byte[] { HXL }, ReadBuf);

                    if (ReadBuf[6] != 0x08)
                    {
                        data[0] = BitConverter.ToInt16(new byte[] { ReadBuf[1], ReadBuf[0] }, 0) * mRes14bits * this.calibrationData[0];
                        data[1] = BitConverter.ToInt16(new byte[] { ReadBuf[3], ReadBuf[2] }, 0) * mRes14bits * this.calibrationData[1];
                        data[2] = BitConverter.ToInt16(new byte[] { ReadBuf[5], ReadBuf[4] }, 0) * mRes14bits * this.calibrationData[2];

                    callback(data);
                        Task.Delay(delay).Wait();
                    }
                }
            }
        }
    }
}