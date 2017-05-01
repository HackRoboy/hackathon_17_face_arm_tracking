using System;
using System.Windows;
using System.IO;
using System.Drawing;
using System.Threading;
using System.Windows.Media;
using System.Windows.Controls;
using System.Diagnostics;

using Rosbridge.Client;

using System.Globalization;

using Newtonsoft.Json;
using System.Text;
using Newtonsoft.Json.Linq;
using System.Threading.Tasks;
using System.Windows.Media.Imaging;

namespace rosbridge_client_example_wpf
{
    /// <summary>
    /// Interaction logic for PublisherWindow.xaml
    /// </summary>
    public partial class PublisherWindow : Window, IChildWindow
    {
        private PXCMSession session;
        private PXCMSenseManager senseManager;
        private PXCMFaceData faceData;
        private Thread update;
        private string alertMsg;
        private int cWidth = 640;
        private int cHeight = 480;

        private Publisher _publisher;
        private MessageDispatcher _md;
        private String host = "ws://10.42.0.1:9090";

        public PublisherWindow(Publisher publisher)
        {
            InitializeComponent();

            _publisher = publisher;

            // Start SenseManager and configure the face module
            ConfigureRealSense();

            // Start the Update thread
            update = new Thread(new ThreadStart(Update));
            update.Start();
        }

        private async void Window_Loaded(object sender, RoutedEventArgs e)
        {
            // TopicLabel.Content = "Publish to \"" + _publisher.Topic.Replace("_", "__") + "\" (" + _publisher.Type.Replace("_", "__") + ")";

            await _publisher.AdvertiseAsync();
        }

        private async void PublishButton_Click(object sender, RoutedEventArgs e)
        {
            // var obj = JObject.Parse(MessageTextBox.Text);

            // await _publisher.PublishAsync(obj);
        }

        public async Task CleanUp()
        {
            if (null != _publisher)
            {
                await _publisher.UnadvertiseAsync();
                _publisher = null;
            }
        }

        private async void Window_Unloaded(object sender, RoutedEventArgs e)
        {
            await CleanUp();
        }

        private void ConfigureRealSense()
        {
            PXCMFaceModule faceModule;
            PXCMFaceConfiguration faceConfig;

            // Start the SenseManager and session  
            session = PXCMSession.CreateInstance();
            senseManager = session.CreateSenseManager();

            // Enable the color stream
            senseManager.EnableStream(PXCMCapture.StreamType.STREAM_TYPE_COLOR, cWidth, cHeight, 30);

            // Enable the face module
            senseManager.EnableFace();
            faceModule = senseManager.QueryFace();
            faceConfig = faceModule.CreateActiveConfiguration();

            // Configure for 3D face tracking
            faceConfig.SetTrackingMode(PXCMFaceConfiguration.TrackingModeType.FACE_MODE_COLOR_PLUS_DEPTH);

            // Known issue: Pose isEnabled must be set to false for R200 face tracking to work correctly
            faceConfig.pose.isEnabled = false;

            // Track faces based on their appearance in the scene
            faceConfig.strategy = PXCMFaceConfiguration.TrackingStrategyType.STRATEGY_APPEARANCE_TIME;

            // Set the module to track four faces in this example
            faceConfig.detection.maxTrackedFaces = 1;

            // Enable alert monitoring and subscribe to alert event hander
            faceConfig.EnableAllAlerts();
            faceConfig.SubscribeAlert(FaceAlertHandler);

            // Apply changes and initialize
            faceConfig.ApplyChanges();
            senseManager.Init();
            faceData = faceModule.CreateOutput();

            // Mirror the image
            senseManager.QueryCaptureManager().QueryDevice().SetMirrorMode(PXCMCapture.Device.MirrorMode.MIRROR_MODE_HORIZONTAL);

            // Release resources
            faceConfig.Dispose();
            faceModule.Dispose();
        }

        private Int32 facesDetected = 0;
        private Int32 faceH = 0;
        private Int32 faceW = 0;
        private Int32 faceX = 0;
        private Int32 faceY = 0;
        private float faceDepth = 0;

        /**
	     * vertical field of view of the realsense
	     */
        public static double vfov = 59;

        /**
         * horizontal field of view of the realsense
         */
        public static double hfov = 70;


        private Timer faceInformationSender = null;

        private void runShellCmd(String command)
        {
            int exitCode;
            ProcessStartInfo processInfo;
            Process process;

            processInfo = new ProcessStartInfo("cmd.exe", "/c " + command);
            processInfo.CreateNoWindow = true;
            processInfo.UseShellExecute = false;
            // *** Redirect the output ***
            processInfo.RedirectStandardError = true;
            processInfo.RedirectStandardOutput = true;

            process = Process.Start(processInfo);
            process.WaitForExit();

            // *** Read the streams ***
            // Warning: This approach can lead to deadlocks, see Edit #2
            string output = process.StandardOutput.ReadToEnd();
            string error = process.StandardError.ReadToEnd();

            exitCode = process.ExitCode;
            alertMsg += "output>>" + (String.IsNullOrEmpty(output) ? "(none)" : output);
            alertMsg += ("error>>" + (String.IsNullOrEmpty(error) ? "(none)" : error));
            alertMsg += ("ExitCode: " + exitCode.ToString());

            process.Close();
        }

        private void FaceAlertScheduleCallback(Object stateInfo)
        {
            double width = (double)cWidth;
            double height = (double)cHeight;
            double faceWidth = (double)faceW;
            double faceHeight = (double)faceH;
            int azimuth = (int)((hfov / 2) * (faceX + faceWidth / 2 - width / 2) / (width / 2));
            int elevation = (int)(-(vfov / 2) * (faceY + faceHeight / 2 - height / 2) / (height / 2));

            var obj = JObject.Parse("{\"x\": " + azimuth.ToString(CultureInfo.InvariantCulture) + ",\"y\": " + elevation.ToString(CultureInfo.InvariantCulture) + ",\"z\": " + faceDepth + "}");

            _publisher.PublishAsync(obj);
            // send azimuth, elevation, distance coordinates to ros main
            // runShellCmd("java -jar CameraBridge.jar " + host + " " + azimuth + " " + elevation + " " + faceDepth);
            // new RosBridgeDotNet(host).Publish("/balloonPosition", "geometry_msgs/Point32", new RosBridgeDotNet.Point3D(0, 1, 2));

        }

        int counter = 0;

        private void FaceAlertHandler(PXCMFaceData.AlertData alert)
        {
            int period = 1000;
            AutoResetEvent autoEvent = new AutoResetEvent(false);

            if (faceInformationSender == null)
            {
                // initialize timer, which can be only started manually
                faceInformationSender = new Timer(this.FaceAlertScheduleCallback, autoEvent, Timeout.Infinite, period);
            }

            if (alert.label == PXCMFaceData.AlertData.AlertType.ALERT_NEW_FACE_DETECTED)
            {
                xx = "Fired" + counter++;
                // start timer intermediately
                faceInformationSender.Change(0, period);

            }
            if (alert.label == PXCMFaceData.AlertData.AlertType.ALERT_FACE_LOST)
            {
                xx = "Lost";
                // clear interval and destroy timer
                faceInformationSender.Change(Timeout.Infinite, period);
            }
            alertMsg = alert.label.ToString();
        }

        private void Update()
        {
            // Start AcquireFrame-ReleaseFrame loop
            while (senseManager.AcquireFrame(true) >= pxcmStatus.PXCM_STATUS_NO_ERROR)
            {
                // Acquire color image data
                PXCMCapture.Sample sample = senseManager.QuerySample();
                Bitmap colorBitmap;
                PXCMImage.ImageData colorData;
                sample.color.AcquireAccess(PXCMImage.Access.ACCESS_READ, PXCMImage.PixelFormat.PIXEL_FORMAT_RGB32, out colorData);
                colorBitmap = colorData.ToBitmap(0, sample.color.info.width, sample.color.info.height);

                // Acquire face data
                if (faceData != null)
                {
                    faceData.Update();
                    facesDetected = faceData.QueryNumberOfDetectedFaces();

                    if (facesDetected > 0)
                    {
                        // Get the first face detected (index 0)
                        PXCMFaceData.Face face = faceData.QueryFaceByIndex(0);

                        // Retrieve face location data
                        PXCMFaceData.DetectionData faceDetectionData = face.QueryDetection();
                        if (faceDetectionData != null)
                        {
                            PXCMRectI32 faceRectangle;
                            faceDetectionData.QueryBoundingRect(out faceRectangle);
                            faceH = faceRectangle.h;
                            faceW = faceRectangle.w;
                            faceX = faceRectangle.x;
                            faceY = faceRectangle.y;

                            // Get average depth value of detected face
                            faceDetectionData.QueryFaceAverageDepth(out faceDepth);
                        }
                    }
                }
                
                // Update UI
                Render(colorBitmap, facesDetected, faceH, faceW, faceX, faceY, faceDepth);

                // Release the color frame
                colorBitmap.Dispose();
                sample.color.ReleaseAccess(colorData);
                
                // Render(facesDetected, faceH, faceW, faceX, faceY, faceDepth);
                senseManager.ReleaseFrame();
            }
        }

        private String xx;

        private void Render(Bitmap bitmap, Int32 count, Int32 h, Int32 w, Int32 x, Int32 y, float depth)
        {
            BitmapImage bitmapImage = ConvertBitmap(bitmap);

            if (bitmapImage != null)
            {
                // Update the UI controls
                this.Dispatcher.Invoke(System.Windows.Threading.DispatcherPriority.Normal, new Action(delegate ()
                {
                    // Update the bitmap image
                    imgStream.Source = bitmapImage;

                    // Update the data labels
                    lblFacesDetected.Content = string.Format("Faces Detected: {0}", count);
                    lblFaceH.Content = string.Format("Face Height: {0}", h);
                    lblFaceW.Content = string.Format("Face Width: {0}", w);
                    lblFaceX.Content = string.Format("Face X Coord: {0}", x);
                    lblFaceY.Content = string.Format("Face Y Coord: {0}", y);
                    lblFaceDepth.Content = string.Format("Face Depth: {0}", depth);
                    lblFaceAlert.Content = string.Format("Last Alert: {0}", alertMsg);



                    // Show or hide the face marker
                    if (count > 0)
                    {
                        // Show face marker
                        rectFaceMarker.Height = h;
                        rectFaceMarker.Width = w;
                        Canvas.SetLeft(rectFaceMarker, x);
                        Canvas.SetTop(rectFaceMarker, y);
                        rectFaceMarker.Visibility = Visibility.Visible;
                    }
                    else
                    {
                        // Hide the face marker
                        rectFaceMarker.Visibility = Visibility.Hidden;
                    }
                }));
            }
        }

        private BitmapImage ConvertBitmap(Bitmap bitmap)
        {
            BitmapImage bitmapImage = null;

            if (bitmap != null)
            {
                MemoryStream memoryStream = new MemoryStream();
                bitmap.Save(memoryStream, System.Drawing.Imaging.ImageFormat.Bmp);
                memoryStream.Position = 0;
                bitmapImage = new BitmapImage();
                bitmapImage.BeginInit();
                bitmapImage.StreamSource = memoryStream;
                bitmapImage.CacheOption = BitmapCacheOption.OnLoad;
                bitmapImage.EndInit();
                bitmapImage.Freeze();
            }

            return bitmapImage;
        }

        private void Window_Closing(object sender, System.ComponentModel.CancelEventArgs e)
        {
            ShutDown();
        }

        private void btnExit_Click(object sender, RoutedEventArgs e)
        {
            ShutDown();
            this.Close();
        }

        private void ShutDown()
        {
            // Stop the Update thread
            update.Abort();

            // Dispose RealSense objects
            faceData.Dispose();
            senseManager.Dispose();
            session.Dispose();
            // _md.StopAsync();
        }
    }
}
