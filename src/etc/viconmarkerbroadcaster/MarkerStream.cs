using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading;
using System.Net;
using System.Net.Sockets;
using System.Diagnostics;
using ViconDataStreamSDK.DotNET;

namespace ViconMarkerBroadcaster
{
    /// <remarks>
    /// This class contains minimal functionality, used only to stream labeled 
    /// and unlabeled marker data from Vicon over UDP to another program on 
    /// the local computer. This was made to work with the LTLMoP map creator.
    /// 
    /// All input arguments are optional.
    /// The port for UDP communication can be specified as the first argument 
    /// when calling this program. Otherwise it will default to port 7500.
    /// The frequency for UDP communication can be specified as the second 
    /// argument when calling the program. This should match up with the update
    /// frequency of the ViconMarkerListener in the LTLMoP map creator. If this
    /// is not specified, it will default to 20 Hz.
    /// 
    /// Most of the code is copied and adjusted from DEASL.Components.MapEditor 
    /// and DEASL.Resources.ViconBroadcaster.
    /// </remarks>
    class MarkerStream
    {
        static void Main(string[] args)
        {
            // Default update frequency for sending data to LTLMoP (Hz)
            double DEFAULT_FREQ = 20.0;

            // Default port used for communicating with the LTLMoP map creator.
            int DEFAULT_PORT = 7500;

            // Delay before declaring timeout if no data (ms)
            int timeoutms = 5000;

            // Process arguments
            int port = DEFAULT_PORT;
            double updateFreq = DEFAULT_FREQ;
            if (args.Length > 0)
            {
                // Get port number
                try
                {
                    port = Int32.Parse(args[0]);
                }
                catch (Exception)
                {
                    Console.WriteLine("Invalid argument for port number.");
                    Console.WriteLine("Using default port: " + DEFAULT_PORT);
                }

                // Get update frequency
                if (args.Length > 1)
                {
                    try
                    {
                        updateFreq = Double.Parse(args[1]);
                    }
                    catch (Exception)
                    {
                        Console.WriteLine("Invalid argument for update frequency.");
                        Console.WriteLine("Using default frequency: " + DEFAULT_FREQ);
                    }
                }
            }
            int delayms = (int)(1000.0 / updateFreq);

            // Stopwatch for checking timeout
            Stopwatch swatch = new Stopwatch();

            // Connect to Vicon
            Client vicon = new Client();    // SDK class that controls communication with Vicon
            Output_Connect res = vicon.Connect("10.0.0.102");
            if (res.Result != Result.Success)
            {
                Console.WriteLine("Error while connecting to Vicon.");
                Console.ReadKey();
                vicon = null;
                return;
            }

            // Set stream mode to "polling" of catched Vicon frames
            if (vicon.SetStreamMode(StreamMode.ClientPullPreFetch).Result != Result.Success)
            {
                Console.WriteLine("Error while setting stream mode.");
                Console.ReadKey();
                return;
            }

            // Use unlabeled as well as labeled markers
            // TODO: Make sure that labeled markers are read by default
            if (vicon.EnableUnlabeledMarkerData().Result != Result.Success)
            {
                Console.WriteLine("Error while enabling unlabeled markers.");
                Console.WriteLine("Press any key to exit");
                Console.ReadKey();
                return;
            }

            Console.WriteLine("Connecting to Vicon.");

            // Set up port for connecting to LTLMoP
            UdpClient controlClient = new UdpClient();
            IPEndPoint destAddr = new IPEndPoint(IPAddress.Loopback, port);

            Console.WriteLine("Requesting broadcasting to begin.");

            // Update marker information and send over UDP until closed
            swatch.Start();
            while (true)
            {
                // Update vicon markers
                Result frameResult = vicon.GetFrame().Result;
                uint nMarkers = vicon.GetUnlabeledMarkerCount().MarkerCount;
                if (frameResult == Result.NoFrame || (swatch.IsRunning && nMarkers == 0))
                {
                    if (swatch.IsRunning && swatch.ElapsedMilliseconds > timeoutms)
                    {
                        Console.WriteLine("No data received from Vicon for several seconds.");
                        Console.WriteLine("Please check that Vicon is on and Vicon Nexus is started.");
                        Console.WriteLine("Press any key to exit");
                        Console.ReadKey();
                        return;
                    }
                    continue;
                }
                else if (frameResult != Result.Success)
                {
                    Console.WriteLine("Error while enabling retrieving frame.");
                    continue;
                }

                // Have received data so stop stopwatch
                if (swatch.IsRunning)
                {
                    Console.WriteLine("Connection to Vicon confirmed. Data broadcasting.");
                    swatch.Stop();
                }
                // Loop through frame, retrieving marker positions and adding 
                // them to the byte-message to be sent over UDP
                byte[] message = new byte[nMarkers * 16]; // 8 bytes, per x and y coordinate, per marker
                for (uint i = 0; i < nMarkers; i++)
                {
                    double[] pos = vicon.GetUnlabeledMarkerGlobalTranslation(i).Translation;
                    // Convert millimeters into meters, round to nearest millimeter, then convert to bytes
                    byte[] x = BitConverter.GetBytes(Math.Round(pos[0] / 1000, 3));
                    byte[] y = BitConverter.GetBytes(Math.Round(pos[1] / 1000, 3));
                    System.Buffer.BlockCopy(x, 0, message, (int)i * 16, 8);
                    System.Buffer.BlockCopy(y, 0, message, (int)i * 16 + 8, 8);
                }

                // Send message over UDP
                controlClient.Send(message, message.Length, destAddr);

                // Sleep until next update
                Thread.Sleep(delayms);
            }
        }
    }
}
