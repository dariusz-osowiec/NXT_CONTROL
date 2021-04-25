using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;
using System.Threading;
using AForge.Robotics.Lego;
using SlimDX.DirectInput;
using Microsoft.Kinect;

namespace NXT_CONTROLL_FINAL
{
    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        public MainWindow()
        {
            InitializeComponent();
            MESSAGE_BOX.Items.Add("Przed połączeniem z robotem NXT, upewnij się, że podłączony jest Kinect, Pad oraz nadajnik Bluetooth");
            MESSAGE_BOX.Items.Add("Aby bez przeszkód korzystać z kinecta, należy oddalić się od niego o 1,70 metra do 2 metrów");
            read_file();
        }

        //Zmienne wykorzystywane do obsługi robota NXT
        static public NXTBrick BRICK = new NXTBrick();
        NXTBrick.MotorState LeftMotorState = new NXTBrick.MotorState();
        NXTBrick.MotorState RightMotorState = new NXTBrick.MotorState();

        private void TEST_NXT_Click(object sender, RoutedEventArgs e)
        {
            //Zmienne, do których zapisywane są dane o adresie sprzętowym robota oraz poziomie sygnału
            byte[] address; int signal;
            //Zmienne, do których zapisywane są informacje o robocie - nazwa, ilość wolnej pamięci oraz ilość energii
            string name; int free_memory; int power;
            //Połączenie się z robotem NXT
            try
            {
                BRICK.Connect(COMINFO.Text); //uaktywnia połączenie na porcie COM podanym w polu tekstowym
                BRICK.PlayTone(600, 300); //wymuszenie sygnału dźwiękowego na robocie, dla potwierdzenia połączenia
                MESSAGE_BOX.Items.Add("Nazwiązano połączenie..."); //dodanie do listy odpowiedniej informacji
                BRICK.GetBatteryPower(out power); //Uzyskanie informacji o poziomie baterii
                //uzyskanie informacji o nazwie robota, poziomie sygnału, adresie sprzętowym oraz ilości wolnej pamięci
                BRICK.GetDeviceInformation(out name, out address, out signal, out free_memory);
                //Wyświetlenie uzyskanych informacji w polu tekstowym
                MESSAGE_BOX.Items.Add("Połączono z robotem LEGO NXT o nazwie: " + name);
                MESSAGE_BOX.Items.Add("Baterie naładowane w " + power / 100 + "%");
               MESSAGE_BOX.Items.Add("Na kostce jest " + free_memory + "KB wolnej pamięci");
            }
            catch
            {
                //W razie braku połączenia z robotem, wyświetlana jest odpowiednia informacja
                MESSAGE_BOX.Items.Add("Błąd połączenia Bluetooth. Sprawdź port COM, urządzenie bluetooth oraz robota NXT");
            }
        }

        //Funkcja ustawiająca wstępnie motory robota NXT
        private void NXTMotorSet()
        {
            //Ustawienie wartości domyślnych serwomotorów robota LEGO NXT
            //ustawienie mocy lewego serwomotoru
            LeftMotorState.Power = 0;
            //właczenie lewego serwomotoru
            LeftMotorState.Mode = NXTBrick.MotorMode.On;
            //ustawienie trybu regulacji
            LeftMotorState.Regulation = NXTBrick.MotorRegulationMode.Sync;
            //Ustawienie lewego serwomotoru na ruch
            LeftMotorState.RunState = NXTBrick.MotorRunState.Running;
            //ustawienie mocy prawego serwomotoru
            RightMotorState.Power = 0;
            //właczenie prawego serwomotoru
            RightMotorState.Mode = NXTBrick.MotorMode.On;
            //ustawienie trybu regulacji
            RightMotorState.Regulation = NXTBrick.MotorRegulationMode.Sync;
            //Ustawienie lewego serwomotoru na ruch
            RightMotorState.RunState = NXTBrick.MotorRunState.Running;
            //Zapisanie informacji na mikrokontrolerze robota LEGO NXT
            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
        }

        //Sekcja zmiennych obsługiwanych w przypadku sterowania padem
        Joystick PAD;
        JoystickState PAD_STATE;
        bool[] buttons;
        int[] axes = new int[4]; 

        private void PAD_CONTROLL_Click(object sender, RoutedEventArgs e)
        {
            GetPad();
            if (BRICK.IsConnected == true)
            {
                NXTMotorSet();
                NXT_PAD_Drive();
                NXT_PAD_Dispose();
            }
            else
            {
                MESSAGE_BOX.Items.Add("Nie połączono z robotem LEGO NXT");
            }
        }


        private void GetPad()
        {
            //konfiguracja PADA
            DirectInput dinput = new DirectInput();
            //Szukanie pada
            foreach (DeviceInstance device in dinput.GetDevices(DeviceClass.GameController, DeviceEnumerationFlags.AttachedOnly))
            {
                try
                {
                    PAD = new Joystick(dinput, device.InstanceGuid);
                    PAD.Acquire();
                    Capabilities cap = PAD.Capabilities;
                    MESSAGE_BOX.Items.Add("Znaleziono Pada!!!");
                    dinput.Dispose();
                    //Wstępne pozyskanie informacji o przyciskach 
                    PadUpdate();
                }
                catch
                {
                    MESSAGE_BOX.Items.Add("Błąd Pada!!!");
                }
            }
        }

        //Funkcja uzyskująca dane o przyciskach i stanach gałek analogowych
        private void PadUpdate()
        {
            PAD.Poll();
            PAD_STATE = PAD.GetCurrentState();
            buttons = PAD_STATE.GetButtons();
            axes[0] = (PAD_STATE.X - 32767) / 435;
            axes[1] = (PAD_STATE.Y - 32767) / -325;
            axes[2] = (PAD_STATE.RotationZ - 32767) / 325;
            axes[3] = (PAD_STATE.Z - 32767) / -325;
        }

        //Funkcja umożliwiająca sterowanie robotem NXT za pomocą pada
        public void NXT_PAD_Drive()
        {
            BRICK.PlayTone(600, 600);
            while (buttons[9] == false)
            {
                PadUpdate();
                LeftMotorState.Power =  axes[1];
                RightMotorState.Power = axes[3];
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
            }
        }

        //Wyłączenie pada
        private void NXT_PAD_Dispose()
        {
            BRICK.PlayTone(500, 400);
            //Wyłączenie motorów
            LeftMotorState.RunState = NXTBrick.MotorRunState.Idle;
            RightMotorState.RunState = NXTBrick.MotorRunState.Idle;
            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
            PAD.Unacquire();
            PAD.Dispose();
            //Rozłączenie z NXT
            MESSAGE_BOX.Items.Add("Pad odłączony!!!");
            
        }

        int LeftMotorPower = 0, RightMotorPower = 0;
        private KinectSensor Kinect; //zmienna "uchwytu" Kinecta
        const float RenderWidth = 640.0f;
        const float RenderHeight = 480.0f;
        const double JointThickness = 3;
        const double BodyCenterThickness = 10;
        const double ClipBoundsThickness = 10;
        readonly Brush centerPointBrush = Brushes.Blue;
        readonly Brush trackedJointBrush = new SolidColorBrush(Color.FromArgb(255, 68, 192, 68));
        readonly Brush inferredJointBrush = Brushes.Yellow;
        readonly Pen trackedBonePen = new Pen(Brushes.Green, 6);
        readonly Pen inferredBonePen = new Pen(Brushes.Gray, 1);
        DrawingGroup drawingGroup;
        DrawingImage imageSource;
        //zmienne przechowujące obliczone informacje o punkcie X i Y
        int PointMeasureY = 0; int PointMeasureX = 0;
        //Listy przechowujace odpowiednio informacje o punkcie X i Y
        public List<double> PMX = new List<double>();
        public List<double> PMY = new List<double>();
        int i = 0, z = 0; //licznik, dzięki któremu poruszamy się po liście

        private void GetKinect()
        {
            drawingGroup = new DrawingGroup();
            imageSource = new DrawingImage(this.drawingGroup);
            Image.Source = this.imageSource;
            //sekcja odpowiadająca za znalezienie pierwszego podłączonego urządzenia Kinect do komputera
            foreach (var potentialSensor in KinectSensor.KinectSensors)
            {
                if (potentialSensor.Status == KinectStatus.Connected)
                {
                    //jeżeli takie urządzenie zostanie wykryte, zostaje utworzony tzw. "uchwyt" urządzenia
                    this.Kinect = potentialSensor;
                    MESSAGE_BOX.Items.Add("Znaleziono Kinecta!!!");
                    break; //
                }
            }
            if (null != this.Kinect) //jeżeli "uchwyt" istnieje i jest podłączone urządzenie Kinect
            {
                // Ustawienie początkowe kinecta - śledzenie szkieletu oraz tryb siedzący
                this.Kinect.SkeletonStream.Enable();
                //Właczenie trybu siedzącego, zakładając, że operator robota będzie sterował urządzeniem siedząc
                //Pozwala też na zaoszczędzenie pamięci, ponieważ Kinect teraz skanuje "przeguby" od pasa w górę
                this.Kinect.SkeletonStream.TrackingMode = SkeletonTrackingMode.Seated;
                // Odbieranie nowej klatki z urządzenia Kinect (włączony tryb siedzący)
                if (Kinect_DRIVE.IsChecked == true ) //tryb sterowania bezpośredniego robotem LEGO NXT
                    Kinect.SkeletonFrameReady += SensorSkeletonFrameReady_ControlMode;
                if (Kinect_TRACE.IsChecked == true ) //tryb sterowania za pomocą pokazanej trajektorii
                    Kinect.SkeletonFrameReady += SensorSkeletonFrameReady_TrackingMode;
                //Uruchomienie sensora
                try
                {
                    //wystartowanie procesu kinecta działającego w tle
                    this.Kinect.Start();
                    MESSAGE_BOX.Items.Add("Uruchomiono Kinecta!!!");
                }
                catch
                {
                    this.Kinect = null;
                }
            }    
        }

        //Inicjalizacja Kinecta
        private void KINECT_CONTROLL_Click(object sender, RoutedEventArgs e)
        {
                //uruchomnienie funkcji sterujących robotem NXT
             if (BRICK.IsConnected == true)
             {
                 NXTMotorSet();
                 GetKinect();
             }
             else
             {
                    MESSAGE_BOX.Items.Add("Nie połączono z robotem LEGO NXT");
             } 
        }

        private void SensorSkeletonFrameReady_ControlMode(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0]; //stworzenie tablicy zbierającej dane o całym zarejestrowanym szkielecie
            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength]; //nadanie rozmiaru tablicy szkieletu
                    skeletonFrame.CopySkeletonDataTo(skeletons); //skopiowanie danych zarejestrowanego szkieletu z klatki
                }
            }
            if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons) //znalezienie danych o szkielecie
                    {
                        if (skel.TrackingState == SkeletonTrackingState.Tracked) //przefiltrowanie tylko tych, które są śledzone
                        {
                            foreach (Joint joint in skel.Joints) //wyodrębnienie danych o przegubach - nadbgarstek, łokieć itp. 
                            {
                                if ((joint.TrackingState == JointTrackingState.Tracked) && ((joint.JointType == JointType.HandLeft) || (joint.JointType == JointType.HandRight)))
                                {
                                    if (joint.JointType == JointType.HandLeft) //sterowanie lewego serwomotoru za pomocą lewej ręki
                                    {
                                        RightMotorPower = Convert.ToInt32(joint.Position.Y * 1000) / 10; 
                                        //przełożenie odczytanej wartości na liczby z zakresu -100 do 100
                                        if (RightMotorPower <= 50 && RightMotorPower >= -50)
                                        {
                                            //jeżeli przeliczona wartość mieści sie w zakresie, pomnóż ją razy 4, aby móc poruszać serwomotorem
                                            RightMotorState.Power = RightMotorPower * 4;
                                            //zapisanie informacji o mocy lewego serwomotoru na mikrokontrolerze
                                            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                                        }
                                        else if (RightMotorPower > 50 || RightMotorPower < -50)
                                        {
                                            //Jeżeli przeliczona wartość wychodzi po za zakres, zatrzymaj motor
                                            RightMotorState.Power = 0;
                                            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                                        }
                                    }
                                    else if (joint.JointType == JointType.HandRight) 
                                    {
                                        LeftMotorPower = Convert.ToInt32(joint.Position.Y * 1000) / 10;
                                        //przełożenie odczytanej wartości na liczby z zakresu -100 do 100
                                        if (LeftMotorPower <= 50 && LeftMotorPower >= -50)
                                        {
                                            //jeżeli przeliczona wartość mieści sie w zakresie, pomnóż ją razy 4, aby móc poruszać serwomotorem
                                            LeftMotorState.Power = LeftMotorPower * 4;
                                            //zapisanie informacji o mocy prawego serwomotoru na mikrokontrolerze
                                            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                                        }
                                        else if (LeftMotorPower > 50 || LeftMotorPower < -50)
                                        {
                                            //Jeżeli przeliczona wartość wychodzi po za zakres, zatrzymaj motor
                                            LeftMotorState.Power = 0;
                                            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                                        }
                                    }
                                }
                            }
                        }
                    }
                }
        }

        private void SensorSkeletonFrameReady_TrackingMode(object sender, SkeletonFrameReadyEventArgs e)
        {
            Skeleton[] skeletons = new Skeleton[0];

            using (SkeletonFrame skeletonFrame = e.OpenSkeletonFrame())
            {
                if (skeletonFrame != null)
                {
                    skeletons = new Skeleton[skeletonFrame.SkeletonArrayLength];
                    skeletonFrame.CopySkeletonDataTo(skeletons);
                }
            }

            using (DrawingContext dc = this.drawingGroup.Open())
            {
                // Draw a transparent background to set the render size
                dc.DrawRectangle(Brushes.Black, null, new Rect(0.0, 0.0, RenderWidth, RenderHeight));

                if (skeletons.Length != 0)
                {
                    foreach (Skeleton skel in skeletons)
                    {
                        RenderClippedEdges(skel, dc);

                        if (skel.TrackingState == SkeletonTrackingState.Tracked)
                        {
                            this.DrawJoints(skel, dc);
                        }
                        else if (skel.TrackingState == SkeletonTrackingState.PositionOnly)
                        {
                            dc.DrawEllipse(
                            this.centerPointBrush,
                            null,
                            this.SkeletonPointToScreen(skel.Position),
                            BodyCenterThickness,
                            BodyCenterThickness);
                        }
                    }
                }

                // prevent drawing outside of our render area
                this.drawingGroup.ClipGeometry = new RectangleGeometry(new Rect(0.0, 0.0, RenderWidth, RenderHeight));
            }
        }

        private static void RenderClippedEdges(Skeleton skeleton, DrawingContext drawingContext)
        {
            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Bottom))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, RenderHeight - ClipBoundsThickness, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Top))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, RenderWidth, ClipBoundsThickness));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Left))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(0, 0, ClipBoundsThickness, RenderHeight));
            }

            if (skeleton.ClippedEdges.HasFlag(FrameEdges.Right))
            {
                drawingContext.DrawRectangle(
                    Brushes.Red,
                    null,
                    new Rect(RenderWidth - ClipBoundsThickness, 0, ClipBoundsThickness, RenderHeight));
            }
        }

        private void DrawJoints(Skeleton skeleton, DrawingContext drawingContext)
        {
            foreach (Joint joint in skeleton.Joints)
            {
                Brush drawBrush = null;

                if ((joint.TrackingState == JointTrackingState.Tracked) && (joint.JointType == JointType.HandLeft))
                {
                    drawBrush = this.trackedJointBrush;
                    //Przełożenie odczytanych wartości o punkcie Y na liczbę z zakresu 0 do 100
                    PointMeasureY = (Convert.ToInt32(joint.Position.Y * 1000) / 10) + 50;
                    //Przełożenie odczytanych wartości o punkcie X na liczbę z zakresu 0 do 100
                    PointMeasureX = (Convert.ToInt32(joint.Position.X * 1000) / 10) + 50;
                    //Również dzięki tym obliczeniom, niejako "przenosimy" środek układu współrzędnych w dół
                    //Zapisanie informacji o punkcie X i Y do odpowiadających im list
                    PMX.Add(Convert.ToDouble(PointMeasureX));
                    XTrace.Content = Convert.ToString(PointMeasureX);
                    PMY.Add(Convert.ToDouble(PointMeasureY));
                    YTrace.Content = Convert.ToString(PointMeasureY);
                    i++;
                    //wzrost licznika o 1
                    z++;
                    //zapisanie informacji o punktach do porównania
                }
                if (drawBrush != null)
                {
                    drawingContext.DrawEllipse(drawBrush, null, this.SkeletonPointToScreen(joint.Position), JointThickness, JointThickness);
                }
            }
        }

        private Point SkeletonPointToScreen(SkeletonPoint skelpoint)
        {
            // Convert point to depth space.  
            // We are not using depth directly, but we do want the points in our 640x480 output resolution.
            DepthImagePoint depthPoint = Kinect.CoordinateMapper.MapSkeletonPointToDepthPoint(skelpoint, DepthImageFormat.Resolution640x480Fps30);
            return new Point(depthPoint.X, depthPoint.Y);
        }

        private void NXT_DISCONNECT_Click(object sender, RoutedEventArgs e)
        {
            if (BRICK.IsConnected == true)
            {
                BRICK.PlayTone(100, 100);
                MESSAGE_BOX.Items.Add("Rozłączono z robotem LEGO NXT!");
                BRICK.Disconnect();
            }
            else
            {
                MESSAGE_BOX.Items.Add("Nie połączono z robotem LEGO NXT!");
            }
        }

        private void STOP_KINECT_CONTROLL_Click(object sender, RoutedEventArgs e)
        {
            if(Kinect != null)
            {
                Kinect.Stop();
                if (Kinect_DRIVE.IsChecked == true)
                {
                    BRICK.PlayTone(500, 500);
                    //Wyłączenie motorów
                    LeftMotorState.RunState = NXTBrick.MotorRunState.Idle;
                    RightMotorState.RunState = NXTBrick.MotorRunState.Idle;
                    BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                    BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                    //wyłączenie kinecta
                    Kinect.Stop();
                    MESSAGE_BOX.Items.Add("Odłączono kinecta!");
                }
                if (Kinect_TRACE.IsChecked == true)
                {
                    BRICK.PlayTone(500, 500);
                    //wyłączenie kinecta
                    MESSAGE_BOX.Items.Add("Odłączono kinecta!");
                    MESSAGE_BOX.Items.Add("Filtrowanie próbek");
                    //filtrowanie
                    filter();
                    //zapisanie trajektorii do pliku
                    MESSAGE_BOX.Items.Add("Zapisywanie przefiltrowanych próbek do pliku...");
                    save_to_file_kinect();
                    save_to_file();
                    MESSAGE_BOX.Items.Add("Obliczanie trasy...");
                    //Włączenie funkcji liczącej trasę
                    if (TRACK_MODE.Text == "Metoda 1")
                    {
                        path_calculation_1();
                    }
                    else if (TRACK_MODE.Text == "Metoda 2")
                    {
                        path_calculation_2();
                        save_spline_to_file();
                        XSlist.Clear(); YSlist.Clear();
                    }
                    //czyszczenie listy
                    MESSAGE_BOX.Items.Add("Koniec!");
                    PMX.Clear(); PMY.Clear(); i = 0;
                    NewPMX.Clear(); NewPMY.Clear(); k = 0;
                }
            }
        }

        //Odtwarzanie trasy - kąty i proste 
        private void path_calculation_1()
        {
            double vectorX, vectorY, vectorPX, vectorPY, length, distance, scalar, cos_angle, road;
            int j = 0, q = 0, angle_value = 0;
            vectorX = NewPMX[1] - NewPMX[0];
            vectorY = NewPMY[1] - NewPMY[0];
            length = System.Math.Sqrt(System.Math.Pow(vectorX,2) + System.Math.Pow(vectorY,2));
            while (j + 2 <= k - 1)
            {
                MESSAGE_BOX.Items.Add("ITERACJA " + j + "...");
                vectorPX = NewPMX[j + 2] - NewPMX[j];
                vectorPY = NewPMY[j + 2] - NewPMY[j];
                distance = System.Math.Sqrt(System.Math.Pow(vectorPX, 2) + System.Math.Pow(vectorPY, 2));
                scalar = (vectorX * vectorPX) + (vectorY * vectorPY);
                cos_angle = scalar / (length * distance);
                while (q + 2 <= 181)
                {
                    if ((Angles_Values[q] > cos_angle) && (Angles_Values[q + 2] < cos_angle))
                    {
                        angle_value = q + 1;
                        RightMotorState.TachoLimit = angle_value;
                        LeftMotorState.TachoLimit = angle_value;
                        break;
                    }
                    q++;
                }
                q = 0;         
                //jeżeli współrzędne wektora poprzedniego X i Y będą ujemne, lub X będzie ujemne i Y zerowe, 
                //a współrzędna X wektora docelowego będzie dodatnia, to skręć w lewo.
                if ((vectorY <= 0) && (vectorX < 0) && (vectorPX > 0))
                {
                        RightMotorState.Power = 75;
                        LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą ujemne, lub X będzie ujemne i Y zerowe, 
                //a współrzędna X wektora docelowego będzie ujemne, to skręć w prawo.
                else if ((vectorY <= 0) && (vectorX < 0) && (vectorPX < 0))
                {
                        RightMotorState.Power = -75;
                        LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą dodatnie, lub X będzie dodatnie i Y zerowe  
                //a współrzędna X wektora docelowego będzie ujemna, to skręć w lewo.
                else if ((vectorY >= 0) && (vectorX > 0) && (vectorPX < 0))
                {
                        RightMotorState.Power = 75;
                        LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą dodatnie, lub X będzie dodatnie i Y zerowe, 
                //a współrzędna X wektora docelowego będą dodatnie, to skręć w prawo.
                else if ((vectorY >= 0) && (vectorX > 0) && (vectorPX > 0))
                {
                        RightMotorState.Power = -75;
                        LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest ujemna lub zerowa, 
                //a Y dodatnia i współrzędna Y wektora docelowego będzie ujemna, to skręć w lewo.
                else if ((vectorY >= 0) && (vectorX <= 0) && (vectorPY < 0))
                {
                        RightMotorState.Power = 75;
                        LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest ujemna lub zerowa, a Y dodatnia 
                //i współrzędna Y wektora docelowego będzie dodatnia, to skręć w prawo.
                else if ((vectorY > 0) && (vectorX <= 0) && (vectorPY > 0))
                {
                        RightMotorState.Power = -75;
                        LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest dodatnia lub zerowa, a Y ujemna 
                //i współrzędna Y wektora docelowego będzie ujemna, to skręć w prawo.
                else if ((vectorY < 0) && (vectorX >= 0) && (vectorPY < 0))
                {
                        RightMotorState.Power = -75;
                        LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest dodatnia lub zerowa, a Y ujemna 
                //i współrzędna Y wektora docelowego będzie dodatnia, to skręć w lewo.
                else if ((vectorY < 0) && (vectorX >= 0) && (vectorPY > 0))
                {
                        RightMotorState.Power = 75;
                        LeftMotorState.Power = -75;
                }
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                //przypisz współrzędne wektora docelowego i jego długość do zmiennych przechowujących wartości
                //wektora początkowego i jego długości, ponieważ to w nastęnej iteracji od nich będzie liczony kąt obrotu.
                length = distance;
                vectorX = vectorPX;
                vectorY = vectorPY;
                //Obliczanie kąta w stopniach, czyli określenie, o ile robot musi obrócić swoje koła aby pokonać długość
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                //przypisz wektor i długość wektora do zmiennych, ponieważ to od nich będzie mierzony obrót robota
                length = distance;
                vectorX = vectorPX;
                vectorY = vectorPY;
                //Oblicz długość trasy
                road = (360 * distance)/(2 * System.Math.PI * 2.16); 
                //Prowadź robota do punktu
                RightMotorState.TachoLimit = Convert.ToInt32(road);
                LeftMotorState.TachoLimit = Convert.ToInt32(road);
                RightMotorState.Power = 75;
                LeftMotorState.Power = 75;
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                MESSAGE_BOX.Items.Add("KONIEC ITERACJI " + j + "..."); 
                j++;
            }
            RightMotorState.Power = 0;
            LeftMotorState.Power = 0;
            RightMotorState.TachoLimit = 0;
            LeftMotorState.TachoLimit = 0;
            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
        }

        //Odtwarzanie trasy - wykorzystanie interpolacji splainowej.
        List<float> XSlist = new List<float>();
        List<float> YSlist = new List<float>();
        private void path_calculation_2()
        {
            //tablice, gdzie będą przechowywane początek i koniec interpolowanego odcinka.
            float[] CopyX = new float[1];
            float[] CopyY = new float[1];
            //Zmienne pomocnicze, służące do sortowania punktami interpolowanego odcinka.
            float compareX, helpY;
            //zmienne przechowujące ilość zmiennych, które będą się składały na interpolowany odcinek, oraz licznik.
            int n = 50, q = 0;
            //Tablica zawierająca ułamkowe części odcinka, a konkretniej jego wartości X.
            float[] xs = new float[n];
            //Tworzenie trasy opartej o interpolację splainową
            while (q < k -2)
            {
                //Pobieranie dwóch punktów z tablicy zarejestowanych współrzędnych X i Y, czyli pobranie odcinka do interpolacji.
                CopyX[0] = (float)NewPMX[q];
                CopyY[0] = (float)NewPMY[q];
                CopyX[1] = (float)NewPMX[q + 1];
                CopyY[1] = (float)NewPMY[q + 1];
                //sortowanie bąbelkowe pomiędzy wartościami X.
                for (int l = 0; l < 1; l++)
                {
                        if (CopyX[l] > CopyX[l + 1])
                        {
                            compareX = CopyX[l];
                            helpY = CopyY[l];
                            CopyX[l] = CopyX[l + 1];
                            CopyY[l] = CopyY[l + 1];
                            CopyX[l + 1] = compareX;
                            CopyY[l + 1] = helpY;
                        }
                }
                //Stworzenie tablicy współrzędnych ułamkowych interpolowanego odcinka. 
                float stepSize = (CopyX[CopyX.Length - 1] - CopyX[0]) / (n - 1);
                for (int i = 0; i < n; i++)
                {
                    xs[i] = CopyX[0] + i * stepSize;
                }
                //Stworzenie zmiennej przechowującej splain.
                CubicSpline spline = new CubicSpline();
                //Obliczenie interpolacji splainowej.
                float[] ys = spline.FitAndEval(CopyX, CopyY, xs);
                //Zapisanie powstałego odcinka do listy trzymającej dane o wyznaczonej trajektorii,
                //lecz już po operacji interpolacji.
                for (int i = 0; i < ys.Length-1; i++)
                {
                    XSlist.Add(xs[i]);
                    YSlist.Add(ys[i]);
                }
                q++;
            }
            //Kiedy już została obliczona cała trasa, można ją odtworzyć, korzystając z algorytmu podobnego do tego,
            //który został użyty przy metodzie pierwszej, lecz dla potrzeb drugiej metody został on ulepszony,
            //oraz został skrócony czas pomiędzy wysyłaniem następnych poleceń do robota, dla zachowania większej płynności ruchu
            double vectorX, vectorY, vectorPX, vectorPY, length, distance, scalar, cos_angle, road;
            int angle_value = 0, j = 0;;
            q = 0;
            vectorX = XSlist[1] - XSlist[0];
            vectorY = YSlist[1] - YSlist[0];
            length = System.Math.Sqrt(System.Math.Pow(vectorX, 2) + System.Math.Pow(vectorY, 2));
            while (j + 2 <= k - 1)
            {
                MESSAGE_BOX.Items.Add("ITERACJA " + j + "...");
                vectorPX = XSlist[j + 2] - XSlist[j];
                vectorPY = XSlist[j + 2] - XSlist[j];
                distance = System.Math.Sqrt(System.Math.Pow(vectorPX, 2) + System.Math.Pow(vectorPY, 2));
                scalar = (vectorX * vectorPX) + (vectorY * vectorPY);
                cos_angle = scalar / (length * distance);
                while (q + 2 <= 181)
                {
                    if ((Angles_Values[q] > cos_angle) && (Angles_Values[q + 2] < cos_angle))
                    {
                        angle_value = q + 1;
                        RightMotorState.TachoLimit = angle_value;
                        LeftMotorState.TachoLimit = angle_value;
                        break;
                    }
                    q++;
                }
                q = 0;
                //jeżeli współrzędne wektora poprzedniego X i Y będą ujemne, lub X będzie ujemne i Y zerowe, 
                //a współrzędna X wektora docelowego będzie dodatnia, to skręć w lewo.
                if ((vectorY <= 0) && (vectorX < 0) && (vectorPX > 0))
                {
                    RightMotorState.Power = 75;
                    LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą ujemne, lub X będzie ujemne i Y zerowe, 
                //a współrzędna X wektora docelowego będzie ujemne, to skręć w prawo.
                else if ((vectorY <= 0) && (vectorX < 0) && (vectorPX < 0))
                {
                    RightMotorState.Power = -75;
                    LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą dodatnie, lub X będzie dodatnie i Y zerowe  
                //a współrzędna X wektora docelowego będzie ujemna, to skręć w lewo.
                else if ((vectorY >= 0) && (vectorX > 0) && (vectorPX < 0))
                {
                    RightMotorState.Power = 75;
                    LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X i Y będą dodatnie, lub X będzie dodatnie i Y zerowe, 
                //a współrzędna X wektora docelowego będą dodatnie, to skręć w prawo.
                else if ((vectorY >= 0) && (vectorX > 0) && (vectorPX > 0))
                {
                    RightMotorState.Power = -75;
                    LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest ujemna lub zerowa, 
                //a Y dodatnia i współrzędna Y wektora docelowego będzie ujemna, to skręć w lewo.
                else if ((vectorY >= 0) && (vectorX <= 0) && (vectorPY < 0))
                {
                    RightMotorState.Power = 75;
                    LeftMotorState.Power = -75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest ujemna lub zerowa, a Y dodatnia 
                //i współrzędna Y wektora docelowego będzie dodatnia, to skręć w prawo.
                else if ((vectorY > 0) && (vectorX <= 0) && (vectorPY > 0))
                {
                    RightMotorState.Power = -75;
                    LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest dodatnia lub zerowa, a Y ujemna 
                //i współrzędna Y wektora docelowego będzie ujemna, to skręć w prawo.
                else if ((vectorY < 0) && (vectorX >= 0) && (vectorPY < 0))
                {
                    RightMotorState.Power = -75;
                    LeftMotorState.Power = 75;
                }
                //jeżeli współrzędne wektora poprzedniego X jest dodatnia lub zerowa, a Y ujemna 
                //i współrzędna Y wektora docelowego będzie dodatnia, to skręć w lewo.
                else if ((vectorY < 0) && (vectorX >= 0) && (vectorPY > 0))
                {
                    RightMotorState.Power = 75;
                    LeftMotorState.Power = -75;
                }
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                //przypisz współrzędne wektora docelowego i jego długość do zmiennych przechowujących wartości
                //wektora początkowego i jego długości, ponieważ to w nastęnej iteracji od nich będzie liczony kąt obrotu.
                length = distance;
                vectorX = vectorPX;
                vectorY = vectorPY;
                //Obliczanie kąta w stopniach, czyli określenie, o ile robot musi obrócić swoje koła aby pokonać długość
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                //przypisz wektor i długość wektora do zmiennych, ponieważ to od nich będzie mierzony obrót robota
                length = distance;
                vectorX = vectorPX;
                vectorY = vectorPY;
                //Oblicz długość trasy
                road = (360 * distance) / (2 * System.Math.PI * 2.16);
                //Prowadź robota do punktu
                RightMotorState.TachoLimit = Convert.ToInt32(road);
                LeftMotorState.TachoLimit = Convert.ToInt32(road);
                RightMotorState.Power = 75;
                LeftMotorState.Power = 75;
                BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
                BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
                System.Threading.Thread.Sleep(1000);
                MESSAGE_BOX.Items.Add("KONIEC ITERACJI " + j + "...");
                j++;
            }
            RightMotorState.Power = 0;
            LeftMotorState.Power = 0;
            RightMotorState.TachoLimit = 0;
            LeftMotorState.TachoLimit = 0;
            BRICK.SetMotorState(NXTBrick.Motor.C, RightMotorState);
            BRICK.SetMotorState(NXTBrick.Motor.A, LeftMotorState);
        }

        private void save_spline_to_file()
        {
            int l = 0;
            int size = XSlist.Count;
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"C:\Data\Data_Map_Spline.txt", false))
            {
                while (l < size)
                {
                    file.WriteLine(XSlist[l] + " " + YSlist[l]);
                    l++;
                }
            }
        }

        private void save_to_file()
        {
            int l = 1;
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"C:\Data\Data_Map.txt", false))
            {
                while (l < k)
                {
                    file.WriteLine(NewPMX[l] + " " + NewPMY[l]);
                    l++;
                }
            }
        }

        private void save_to_file_kinect()
        {
            int l = 1;
            using (System.IO.StreamWriter file = new System.IO.StreamWriter(@"C:\Data\Data_Kinect.txt", false))
            {
                while (l < i)
                {
                    file.WriteLine(PMX[l] + " " + PMY[l]);
                    l++;
                }
            }
        }


        public List<double> Angles_Values = new List<double>();
        int a = 1;
        private void read_file()
        {
            string line;
            using (System.IO.StreamReader file = new System.IO.StreamReader(@"C:\Data\Data_Cosinus.txt"))
            {
                do
                {
                    line = file.ReadLine();
                    Angles_Values.Add(Convert.ToDouble(line));
                    a++;
                }
                while(line !=null);
            }
        }

        
        public List<double> NewPMX = new List<double>();
        public List<double> NewPMY = new List<double>();
        int k = 0;
        private void filter()
        {
            //Tymczasowe listy, zawierające dane po filtracji, jeszcze przed wyrzuceniem powtarzających się wpisów
            List<double> TempPMX = new List<double>();
            List<double> TempPMY = new List<double>();
            //liczniki list - dzięki nim, ustalany jest rozmiar list - tymczasowej i wynikowej.
            int j = 1;
            int l = 0;
            //wartości punktów do porównania przy operacji usuwania powtarzających się wpisów
            double pointx = 0, pointy = 0;
            //zmienne do których zapisywane będą granice filtra.
            double granicax1, granicax2, granicay1, granicay2;
            while(j < i -1)
            {
                    //Obliczenie granic dla wartości X listy.
                    granicax1 = (PMX[j-1] - (PMX[j-1]*4/10));
                    granicax2 = (PMX[j-1] + (PMX[j-1]*4/10));
                    //Obliczenie granic dla wartości Y listy.
                    granicay1 = (PMY[j-1] - (PMY[j-1]*4/10));
                    granicay2 = (PMY[j-1] + (PMY[j-1]*4/10));
                    if ((PMX[j] > granicax1) & (PMX[j] < granicax2) && (PMY[j] > granicay1) & (PMY[j] < granicay2))
                    {        
                            //Jeżeli wartość mieści się w przedziale, zapisz ją do tablicy.
                            MESSAGE_BOX.Items.Add(j + " -> Punkt X: " + PMX[j] + " Y: " + PMY[j] + " mieści się w granicy");
                            TempPMX.Add(PMX[j]);
                            TempPMY.Add(PMY[j]);
                            l++;
                    }
                    j++;
            }
            j = 1;
            //Operacja usuwania powtarzających się wpisów
            while (j < l)
            {
                //Zależność, w której to porównuje się ostatnią unikalną zapisaną wartość z obecną próbką.
                if ((TempPMX[j] != pointx) && (TempPMY[j] != pointy))
                {
                    //Jeżeli zależność zostanie spełniona i punkt jest unikalny, zapisywany jest do wynikowej listy.
                    MESSAGE_BOX.Items.Add(j + " -> Punkt X: " + TempPMX[j] + " Y: " + TempPMY[j] + " został zapisany");
                    NewPMX.Add(TempPMX[j]);
                    NewPMY.Add(TempPMY[j]);
                    //Zwiększenie licznika listy - informuje program o rozmiarze tablicy wynikowej.
                    k++;
                    //Zapisanie punktu w celu porównania go z następnymi wpisami.
                    pointx = TempPMX[j];
                    pointy = TempPMY[j];
                }
                j++;
            }
        }

        private void ClearMessageBox_Click(object sender, RoutedEventArgs e)
        {
            MESSAGE_BOX.Items.Clear();
        }
    }
}
