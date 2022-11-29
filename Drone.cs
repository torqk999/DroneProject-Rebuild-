using Sandbox.Game.EntityComponents;
using Sandbox.ModAPI.Ingame;
using Sandbox.ModAPI.Interfaces;
using SpaceEngineers.Game.ModAPI.Ingame;
using System;
using System.Collections;
using System.Collections.Generic;
using System.Collections.Immutable;
using System.Linq;
using System.Text;
using VRage;
using VRage.Collections;
using VRage.Game;
using VRage.Game.Components;
using VRage.Game.GUI.TextPanel;
using VRage.Game.ModAPI.Ingame;
using VRage.Game.ModAPI.Ingame.Utilities;
using VRage.Game.ObjectBuilders.Definitions;
using VRageMath;

namespace IngameScript
{
    partial class Program : MyGridProgram
    {
        // This file contains your actual script.
        //
        // You can either keep all your code here, or you can create separate
        // code files to make your program easier to navigate while coding.
        //
        // In order to add a new utility class, right-click on your project, 
        // select 'New' then 'Add Item...'. Now find the 'Space Engineers'
        // category under 'Visual C# Items' on the left hand side, and select
        // 'Utility Class' in the main area. Name it in the box below, and
        // press OK. This utility class will be merged in with your code when
        // deploying your final script.
        //
        // You can also simply create a new utility class manually, you don't
        // have to use the template if you don't want to. Just do so the first
        // time to see what a utility class looks like.
        // 
        // Go to:
        // https://github.com/malware-dev/MDK-SE/wiki/Quick-Introduction-to-Space-Engineers-Ingame-Scripts
        //
        // to learn more about ingame scripts.

        /*

                static readonly Vector3[] practiceCoords =
        {
            new Vector3(999673.78,999672.76,999883.59),
            new Vector3(999688.76,999706.31,999886.81),
            new Vector3(999661.84,999711.39,999911.27),
            new Vector3(999644.01,999682.37,999908.57)
        };

                 List<Vector3> GenerateSphere(Vector3 origin, float radius, float distance)
        {
            List<Vector3> output = new List<Vector3>();

            int ringCount = (int)((Math.PI * radius) / distance);
            string debug = string.Empty;


            debug += $"sin(pi  /2): {Math.Sin(Math.PI / 2)}";
            debug += $"sin(pi 3/2): {Math.Sin(Math.PI * 3/2)}";

            for (int i = 0; i <= ringCount; i++)
            {
                debug += $"ringIndex: {i}\n";

                double ringRadius = Math.Sin(((double)i / ringCount) * Math.PI) * radius;
                double deltaY = Math.Cos(((double)i / ringCount) * Math.PI) * radius;

                debug += $"ringRadius: {ringRadius}\n";
                debug += $"deltaY: {deltaY}\n";

                int pointCount = (int)((2 * Math.PI * ringRadius) / distance);
                pointCount = (pointCount < 1) ? 1 : pointCount;

                debug += $"pointCoint: {pointCount}\n";

                for (int j = 0; j < pointCount; j++)
                {
                    double deltaZ = Math.Sin(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;
                    double deltaX = Math.Cos(((double)j / pointCount) * (2 * Math.PI)) * ringRadius;

                    Vector3 delta = new Vector3(deltaX, deltaY, deltaZ);
                    Vector3 newVector = origin + delta;
                    output.Add(newVector);
                }
            }

            DroneSurface.WriteText(debug, false);

            return output;
        }

        void TestGen()
        {
            //Echo("yo");
            List<Vector3> testPoints = GenerateSphere(Me.GetPosition(), Radius, Distance);

            string data = string.Empty;
            int count = 0;
            foreach(Vector3 vector in testPoints)
            {
                data += $"GPS:T{count}:{vector.X}:{vector.Y}:{vector.Z}:#FF75C9F1:\n";
                count++;
            }
            Panel.WriteText(data, false);
        }
         */

        #region TODO
        /*  Automatic block detection and intuitive assumptions
         *  Rotational drift caused by Formation's matrix source
         *  GUI
         *  
         */

        #endregion

        #region TERMINOLOGY
        /*
        
        

         */
        #endregion

        #region OLD_SOURCE

        #region test
        List<IMyMotorBase> MotorBases;
        IMyCubeGrid TestGrid;

        public void WhatAreYou()
        {
            ScreenOut("PARTS", "", false);
            foreach (IMyMotorBase next in MotorBases)
            {
                ScreenOut("PARTS", next.CustomName + "\n", true);
            }

            ScreenOut("MATRIX", "", false);

            string[] matrix = TestGrid.WorldMatrix.ToString().Split(':');
            foreach (string next in matrix)
                ScreenOut("MATRIX", next + "\n", true);

        }
        public void DisplayInputValues(IMyShipController control)
        {
            ScreenOut("MOVE", "", false);   // Clear screen
            if (control != null)
            {
                ScreenOut("MOVE", control.CustomName + "\n", true);
                ScreenOut("MOVE", "Mouse-X: " + control.RotationIndicator.X + "\n", true);
                ScreenOut("MOVE", "Mouse-Y: " + control.RotationIndicator.Y + "\n", true);
                ScreenOut("MOVE", "Move-X: " + control.MoveIndicator.X + "\n", true);
                ScreenOut("MOVE", "Move-Y: " + control.MoveIndicator.Y + "\n", true);
                ScreenOut("MOVE", "Move-Z: " + control.MoveIndicator.Z + "\n", true);
            }
            else
                ScreenOut("MOVE", "Invalid Control Block!", true);
        }
        /*
        public void NormalizeGrid_M()
        {
            Vector3D Origin = BasisVectorBlocks[0].GetPosition();
            Vector3D Xpos = BasisVectorBlocks[1].GetPosition();
            Vector3D Ypos = BasisVectorBlocks[2].GetPosition();
            Vector3D Zpos = BasisVectorBlocks[3].GetPosition();

            TurretNormalMatrix[0] = VRageMath.Vector3D.Normalize(Origin - Xpos);
            TurretNormalMatrix[1] = VRageMath.Vector3D.Normalize(Origin - Ypos);
            TurretNormalMatrix[2] = VRageMath.Vector3D.Normalize(Origin - Zpos);
        }
        */
        Vector3D[] MatrixConversion(MatrixD matrix)
        {
            Vector3D[] result = new Vector3D[3];

            /*
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
             * 
             * My Implementation:
             * Indice[0]: Right.x   , Right.y   , Right.z
             * Indice[1]: Forward.x , Forward.y , Forward.z
             * Indice[2]: Down.x    , Down.y    , Down.z      ?? Is this correct or did keen change there matrix layout  ??
             *                                                ?? Which Side is the pitch rotor on? Create logic for this ??
            */

            result[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
            result[1] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);
            result[2] = new Vector3D(-matrix.M21, -matrix.M22, -matrix.M23);

            return result;
        }
        public void DeltaTarget()
        {
            if (TurretYaw != null && TurretSensor != null)
            {
                Vector3D Origin = TurretPitch.GetPosition();
                Vector3D Target = TurretSensor.LastDetectedEntity.Position;
                TargetDeltaVector = Origin - Target;
            }
        }
        public void NormalizeTarget()
        {
            /* X,Y,Z = Normalized Vector Unit Coefficients
             * x,y,z = Delta Target Vector (raw World GPS)
             * a,b,c = Normalized X vector components (relative x,y,z)
             * d,e,f = Normalized Y ''
             * g,h,i = Normalized Z ''
            */

            Vector3D[] NM = TurretNormalMatrix;
            Vector3D NV = new Vector3D(); // new NormalizedVector
            Vector3D DV = TargetDeltaVector;

            // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

            // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
            NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));

            // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
            // Y = (Z(gb - ha) + ya - xb) / (ea - db)

            // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
            NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));

            // X = (x - (Yd + Zg)) / a
            // X = (y - (Ye + Zh)) / b
            // X = (z - (Yf + Zi)) / c

            // X = (x    - ((Y    * d)        + (Z    * g)))      / a
            NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;

            TargetNormalizedVector = NV;
        }
        public void TargetBearings()
        {
            double yawAdjust = 0;
            if (TargetNormalizedVector.Y > 0)
            {
                if (TargetNormalizedVector.X < 0)
                    yawAdjust = 360;
            }
            else
                yawAdjust = 180;

            TargetRelativeYaw = (Math.Atan(TargetNormalizedVector.X / TargetNormalizedVector.Y) * RadToDeg) + yawAdjust;
            TargetRelativePitch = Math.Atan(TargetNormalizedVector.Z / Math.Sqrt(Math.Pow(TargetNormalizedVector.X, 2) + Math.Pow(TargetNormalizedVector.Y, 2))) * RadToDeg;
        }
        public void ResetBearings()
        {
            TurretYaw.SetValue("Velocity", 1);
        }
        public void YawRotationVelocity(IMyMotorStator rotor)
        {
            if (rotor != null)
            {
                double velocity = 0;
                double domainMin = 0;
                double domainMax = 0;
                double magnitude = 0;
                bool currentIsMin;
                int direction = 1;

                double currentAngle = rotor.Angle * RadToDeg;

                ScreenOut("TRACK", "RadToDeg: " + RadToDeg + "\n", true);
                ScreenOut("TRACK", "CurrentAng: " + currentAngle + "\n", true);

                if (currentAngle > 180)
                {
                    domainMax = currentAngle;
                    domainMin = currentAngle - 180;
                    currentIsMin = false;
                }
                else
                {
                    domainMax = currentAngle + 180;
                    domainMin = currentAngle;
                    currentIsMin = true;
                }

                if (TargetRelativeYaw > domainMin && TargetRelativeYaw <= domainMax)
                {
                    if (currentIsMin)
                    {
                        magnitude = TargetRelativeYaw - currentAngle;
                        direction = 1;
                    }
                    else
                    {
                        magnitude = currentAngle - TargetRelativeYaw;
                        direction = -1;
                    }
                }
                if (TargetRelativeYaw > domainMax)
                {
                    if (currentIsMin)
                    {
                        magnitude = (360 - TargetRelativeYaw) + currentAngle;
                        direction = -1;
                    }
                    else
                    {
                        magnitude = TargetRelativeYaw - currentAngle;
                        direction = 1;
                    }
                }
                if (TargetRelativeYaw <= domainMin)
                {
                    if (currentIsMin)
                    {
                        magnitude = currentAngle - TargetRelativeYaw;
                        direction = -1;
                    }
                    else
                    {
                        magnitude = (360 - currentAngle) + TargetRelativeYaw;
                        direction = 1;
                    }
                }


                velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;

                rotor.SetValueFloat("Velocity", (float)velocity);
            }
        }
        public void PitchRotationVelocity(IMyMotorStator rotor)
        {
            if (rotor != null)
            {
                double magnitude;
                double velocity;
                double adjustedPitch;
                double currentPitch;
                int direction = 1;

                currentPitch = rotor.Angle * RadToDeg;
                adjustedPitch = 90 - currentPitch;

                if (adjustedPitch < TargetRelativePitch)
                    direction = -1;

                magnitude = Math.Abs(adjustedPitch - TargetRelativePitch);
                velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;
                rotor.SetValueFloat("Velocity", (float)velocity);
            }
        }

        #endregion
        #region reference

        const string TurretSensorName = "SpotlightSensor";
        const string TurretYawName = "SpotlightYaw";
        const string TurretPitchName = "SpotlightPitch";
        const string TurretFocusName = "SpotlightFocus";
        const string TurretControlName = "SpotlightControl";

        IMySensorBlock TurretSensor;
        IMyMotorStator TurretYaw;
        IMyMotorStator TurretPitch;
        _Turret Sample;

        Vector3D[] TurretNormalMatrix = new Vector3D[3];
        Vector3D TargetDeltaVector = new Vector3D();
        Vector3D TargetNormalizedVector = new Vector3D();

        double TargetRelativeYaw;
        double TargetRelativePitch;
        double VelocityAdjustPower = 1;
        const double RadToDeg = 180 / Math.PI;

        public void ScreenOut(string target, string log, bool append)
        {
            foreach (IMyTextPanel nextPanel in Panels)
                if (nextPanel.CustomName.Contains(target))
                    nextPanel.WriteText(log, append);
        }
        #endregion
        #region automatic

        bool TrackingOn = true;
        public class _Turret
        {
            public string Name;
            public bool bHasTarget;

            public IMySensorBlock Sensor;
            public IMyMotorStator MotorYaw;
            public IMyMotorStator MotorPitch;
            public IMyTerminalBlock Focus;
            public IMyTerminalBlock Controls;
            public IMyTextSurface Debug;

            public Vector3D[] TurretNormalMatrix = new Vector3D[3];
            public Vector3D TargetDeltaVector = new Vector3D();
            public Vector3D TargetNormalizedVector = new Vector3D();

            public string Keyword = string.Empty;
            public MyDetectedEntityType EntityType;
            public MyDetectedEntityInfo Entity;

            public double TargetRelativeYaw = 0;
            public double TargetRelativePitch = 0;
            public double VelocityAdjustPower = 1;

            const double RadToDeg = 180 / Math.PI;

            public _Turret(IMyTextSurface debug, string name, MyDetectedEntityType entityType = MyDetectedEntityType.None)
            {
                Name = name;
                EntityType = entityType;
                Debug = debug;
            }
            public void UpdateTargetParamaters()
            {
                if (Controls == null)
                    return;

                string[] data = Controls.CustomData.Split('\n');

                /* Targeting Instructions:
                 * 
                 * @ = Keyword
                 * # = Type
                */

                foreach (string nextLine in data)
                {
                    char check = nextLine[0];
                    string label = nextLine.Remove(0, 1);

                    switch (check)
                    {
                        case '@':
                            Keyword = label;
                            break;

                        case '#':
                            foreach (MyDetectedEntityType type in Enum.GetValues(typeof(MyDetectedEntityType)))
                            {
                                if (type.ToString().Contains(label))
                                    EntityType = type;
                            }
                            break;
                    }
                }
            }
            public void TurretUpdate()
            {
                if (MotorPitch == null || MotorYaw == null || Sensor == null || Focus == null)
                    return;

                bHasTarget = CheckForTarget();
                ToggleFocus(bHasTarget);

                if (bHasTarget)
                {
                    MatrixConversion();
                    DeltaTarget();
                    NormalizeTarget();
                    //TargetBearings();
                }
                else
                    ResetBearings();

                YawRotationVelocity(MotorYaw);
                PitchRotationVelocity(MotorPitch);

            }
            void ToggleFocus(bool toggle)
            {
                try
                {
                    IMyFunctionalBlock function = (IMyFunctionalBlock)Focus;
                    function.Enabled = toggle;
                }
                catch
                {
                    // Error message here!
                }
            }
            void MatrixConversion()
            {
                /*
                 * Keen Implementation:
                 * Row 1: Right.x , Right.y , Right.z
                 * Row 2: Up.x    , Up.y    , Up.z
                 * Row 3: Back.x  , Back.y  , Back.z
                 * 
                 * My Implementation:
                 * Indice[0]: Right.x   , Right.y   , Right.z
                 * Indice[1]: Forward.x , Forward.y , Forward.z
                 * Indice[2]: Down.x    , Down.y    , Down.z      ?? Is this correct or did keen change there matrix layout ??
                 * 
                 * New Imp:
                 * Indice[0]: Right.x   , Right.y   , Right.z
                 * Indice[1]: Up.x      , Up.y      , Up.z
                 * Indice[2]: Forward.x , Forward.y , Forward.z
                */

                MatrixD matrix = MotorYaw.WorldMatrix;

                TurretNormalMatrix[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
                TurretNormalMatrix[1] = new Vector3D(matrix.M21, matrix.M22, matrix.M23);
                TurretNormalMatrix[2] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);

                //TurretNormalMatrix[0] = new Vector3D(matrix.M11, matrix.M12, matrix.M13);
                //TurretNormalMatrix[1] = new Vector3D(-matrix.M31, -matrix.M32, -matrix.M33);
                //TurretNormalMatrix[2] = new Vector3D(-matrix.M21, -matrix.M22, -matrix.M23);
            }
            void DeltaTarget()
            {
                Vector3D Origin = Focus.GetPosition();
                Vector3D Target = Entity.Position;
                TargetDeltaVector = Origin - Target;
            }
            void NormalizeTarget()
            {
                /* X,Y,Z = Normalized Vector Unit Coefficients
                 * x,y,z = Delta Target Vector (raw World GPS)
                 * a,b,c = Normalized X vector components (relative x,y,z)
                 * d,e,f = Normalized Y ''
                 * g,h,i = Normalized Z ''
                */

                Vector3D[] NM = TurretNormalMatrix;
                Vector3D NV = new Vector3D(); // new NormalizedVector
                Vector3D DV = TargetDeltaVector;

                // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

                // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
                NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));

                // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
                // Y = (Z(gb - ha) + ya - xb) / (ea - db)

                // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
                NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));

                // X = (x - (Yd + Zg)) / a
                // X = (y - (Ye + Zh)) / b
                // X = (z - (Yf + Zi)) / c

                // X = (x    - ((Y    * d)        + (Z    * g)))      / a
                NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;

                TargetNormalizedVector = NV;
            }

            Vector3 NormalizeTarget2(MatrixD S, Vector3 D) // S = sourceBearing, D = targetWorldDelta
            {
                /* X,Y,Z = Normalized Vector Unit Coefficients
                 * x,y,z = Delta Target Vector (raw World GPS)
                 * a,b,c = Normalized X vector components (relative x,y,z)
                 * d,e,f = Normalized Y ''
                 * g,h,i = Normalized Z ''
                 * 
                 * Keen Implementation:
                 * Row 1: Right.x , Right.y , Right.z
                 * Row 2: Up.x    , Up.y    , Up.z
                 * Row 3: Back.x  , Back.y  , Back.z
                */

                //Vector3D[] NM = TurretNormalMatrix;
                Vector3D NV = new Vector3D(); // new NormalizedVector
                                              //Vector3D DV = TargetDeltaVector;

                // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

                // Z = (d       * ((b       * z)    - (c       * y))    + e       * ((c       * x)    - (a       * z))    + f       * ((a       * y)    - (b       * x)))    / (d       * ((b       * i)       - (c       * h))       + e       * ((c       * g)       - (a       * i))       + f       * ((a       * h)       - (b       * g)))
                //NV.Z = (NM[1].X * ((NM[0].Y * DV.Z) - (NM[0].Z * DV.Y)) + NM[1].Y * ((NM[0].Z * DV.X) - (NM[0].X * DV.Z)) + NM[1].Z * ((NM[0].X * DV.Y) - (NM[0].Y * DV.X))) / (NM[1].X * ((NM[0].Y * NM[2].Z) - (NM[0].Z * NM[2].Y)) + NM[1].Y * ((NM[0].Z * NM[2].X) - (NM[0].X * NM[2].Z)) + NM[1].Z * ((NM[0].X * NM[2].Y) - (NM[0].Y * NM[2].X)));
                NV.Z = (S.M21 * ((S.M12 * D.Z) - (S.M13 * D.Y)) + S.M22 * ((S.M13 * D.X) - (S.M11 * D.Z)) + S.M23 * ((S.M11 * D.Y) - (S.M12 * D.X))) / (S.M21 * ((S.M12 * S.M33) - (S.M13 * S.M32)) + S.M22 * ((S.M13 * S.M31) - (S.M11 * S.M33)) + S.M23 * ((S.M11 * S.M32) - (S.M12 * S.M31)));


                // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
                // Y = (Z(gb - ha) + ya - xb) / (ea - db)

                // Y = (Z    * ((h       * c)       - (i       * b))       + (z    * b)       - (y    * c))       / ((f       * b)       - (e       * c))
                //NV.Y = (NV.Z * ((NM[2].Y * NM[0].Z) - (NM[2].Z * NM[0].Y)) + (DV.Z * NM[0].Y) - (DV.Y * NM[0].Z)) / ((NM[1].Z * NM[0].Y) - (NM[1].Y * NM[0].Z));
                NV.Y = (NV.Z * ((S.M32 * S.M13) - (S.M33 * S.M12)) + (D.Z * S.M12) - (D.Y * S.M13)) / ((S.M23 * S.M12) - (S.M22 * S.M13));

                // X = (x - (Yd + Zg)) / a
                // X = (y - (Ye + Zh)) / b
                // X = (z - (Yf + Zi)) / c

                // X = (x    - ((Y    * d)        + (Z    * g)))      / a
                //NV.X = (DV.X - ((NV.Y * NM[1].X) + (NV.Z * NM[2].X))) / NM[0].X;
                NV.X = (D.X - ((NV.Y * S.M21) + (NV.Z * S.M31))) / S.M11;

                return NV;
            }
            void TargetBearingsOLD()
            {
                double yawAdjust = 0;
                if (TargetNormalizedVector.Y > 0)
                {
                    if (TargetNormalizedVector.X < 0)
                        yawAdjust = 360;
                }
                else
                    yawAdjust = 180;

                TargetRelativeYaw = (Math.Atan(TargetNormalizedVector.X / TargetNormalizedVector.Y) * RadToDeg) + yawAdjust;
                TargetRelativePitch = Math.Atan(TargetNormalizedVector.Z / Math.Sqrt(Math.Pow(TargetNormalizedVector.X, 2) + Math.Pow(TargetNormalizedVector.Y, 2))) * RadToDeg;
            }
            void ResetBearings()
            {
                TargetRelativeYaw = 0;
                TargetRelativePitch = 0;
            }
            void YawRotationVelocity(IMyMotorStator rotor)
            {
                if (rotor != null)
                {
                    double velocity = 0;
                    double domainMin = 0;
                    double domainMax = 0;
                    double magnitude = 0;
                    bool currentIsMin;
                    int direction = 1;

                    double currentAngle = rotor.Angle * RadToDeg;

                    //ScreenOut("TRACK", "RadToDeg: " + RadToDeg + "\n", true);
                    //ScreenOut("TRACK", "CurrentAng: " + currentAngle + "\n", true);

                    if (currentAngle > 180)
                    {
                        domainMax = currentAngle;
                        domainMin = currentAngle - 180;
                        currentIsMin = false;
                    }
                    else
                    {
                        domainMax = currentAngle + 180;
                        domainMin = currentAngle;
                        currentIsMin = true;
                    }

                    if (TargetRelativeYaw > domainMin && TargetRelativeYaw <= domainMax)
                    {
                        if (currentIsMin)
                        {
                            magnitude = TargetRelativeYaw - currentAngle;
                            direction = 1;
                        }
                        else
                        {
                            magnitude = currentAngle - TargetRelativeYaw;
                            direction = -1;
                        }
                    }
                    if (TargetRelativeYaw > domainMax)
                    {
                        if (currentIsMin)
                        {
                            magnitude = (360 - TargetRelativeYaw) + currentAngle;
                            direction = -1;
                        }
                        else
                        {
                            magnitude = TargetRelativeYaw - currentAngle;
                            direction = 1;
                        }
                    }
                    if (TargetRelativeYaw <= domainMin)
                    {
                        if (currentIsMin)
                        {
                            magnitude = currentAngle - TargetRelativeYaw;
                            direction = -1;
                        }
                        else
                        {
                            magnitude = (360 - currentAngle) + TargetRelativeYaw;
                            direction = 1;
                        }
                    }


                    velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;

                    rotor.SetValueFloat("Velocity", (float)velocity);
                }
            }
            void PitchRotationVelocity(IMyMotorStator rotor)
            {
                if (rotor != null)
                {
                    double magnitude;
                    double velocity;
                    double adjustedPitch;
                    double currentPitch;
                    int direction = 1;

                    currentPitch = rotor.Angle * RadToDeg;
                    adjustedPitch = 90 - currentPitch;

                    if (adjustedPitch < TargetRelativePitch)
                        direction = -1;

                    magnitude = Math.Abs(adjustedPitch - TargetRelativePitch);
                    velocity = Math.Pow(magnitude, VelocityAdjustPower) * direction;
                    rotor.SetValueFloat("Velocity", (float)velocity);
                }
            }
            bool CheckForTarget()
            {
                List<MyDetectedEntityInfo> entities = new List<MyDetectedEntityInfo>();
                Sensor.DetectedEntities(entities);
                //Entity = entities.Find(x => x.Type == EntityType && x.Name.Contains(Keyword));
                Entity = entities.Find(x => x.Name.Contains(Keyword));
                //int index = entities.FindIndex(x => x.Type == EntityType && x.Name.Contains(Keyword));
                int index = entities.FindIndex(x => x.Name.Contains(Keyword));
                Debug.WriteText("\nIndex: " + index, false);
                return (index > -1) ? true : false;
            }
        }

        public void BlockDetectionS()
        {
            Sample.Sensor = (IMySensorBlock)GetBlock(TurretSensorName);
            Sample.MotorPitch = (IMyMotorStator)GetBlock(TurretPitchName);
            Sample.MotorYaw = (IMyMotorStator)GetBlock(TurretYawName);
            Sample.Focus = GetBlock(TurretFocusName);
            Sample.Controls = GetBlock(TurretControlName);
        }
        public void AutoTrackTest()
        {
            //DisplayTrackingValues();

            if (TrackingOn)
                Sample.TurretUpdate();
        }
        public void DisplayTrackingValues()
        {
            ScreenOut("TRACK", "", false);   // Clear screen
            ScreenOut("TRACK", "Tracking: " + TrackingOn + "\n", true);
            ScreenOut("TRACK", "TargetIdentity: " + Sample.Sensor.LastDetectedEntity.Name + "\n", true);
            ScreenOut("TRACK", "TargetYaw: " + Sample.TargetRelativeYaw + "\n", true);
            ScreenOut("TRACK", "TargetPitch: " + Sample.TargetRelativePitch + "\n", true);
            ScreenOut("TRACK", "Normal-X: " + Sample.TargetNormalizedVector.X + "\n", true);
            ScreenOut("TRACK", "Normal-Y: " + Sample.TargetNormalizedVector.Y + "\n", true);
            ScreenOut("TRACK", "Normal-Z: " + Sample.TargetNormalizedVector.Z + "\n", true);
        }

        #endregion
        #region manual

        // TARGET VARIABLES //

        public string CONTROL = "Ex - Control";
        public string YAW = "Ex - Yaw";
        public string PITCH = "Ex - Pitch";
        public string ARM = "Ex - Arm";

        public float ARM_VELOCITY = 1;
        public int YAW_DIR = -1;
        public float YAW_MAX = 1;
        public int PITCH_DIR = -1;
        public float PITCH_MAX = 1;

        ///////////////////////////

        IMyShipController ArmControl;
        IMyMotorStator YawBase;
        IMyMotorStator PitchBase;
        IMyBlockGroup ArmGroup;

        List<IMyTextPanel> Panels = new List<IMyTextPanel>();
        List<IMyPistonBase> ArmPistons = new List<IMyPistonBase>();

        //IMyCubeGrid TestGrid;

        // METHODS //
        /// Helper Section

        IMyTerminalBlock GetBlock(string name)
        {
            return GridTerminalSystem.GetBlockWithName(name);
        }
        public float Clamp(float value, float min, float max)
        {
            float output;

            if (value < min)
                output = min;
            else if (value > max)
                output = max;
            else
                output = value;

            return output;
        }

        public void BlockDetectionM()
        {
            Panels.Clear();
            GridTerminalSystem.GetBlocksOfType(Panels);

            //TestGrid = Panels[0].CubeGrid;
        }
        public void BlockAssignmentM()
        {
            List<IMyShipController> controls = new List<IMyShipController>();
            GridTerminalSystem.GetBlocksOfType(controls);
            foreach (IMyShipController next in controls)
                next.IsMainCockpit = false;
            ArmControl = (IMyShipController)GetBlock(CONTROL);
            ArmControl.IsMainCockpit = true;
            YawBase = (IMyMotorStator)GetBlock(YAW);
            PitchBase = (IMyMotorStator)GetBlock(PITCH);
            ArmGroup = GridTerminalSystem.GetBlockGroupWithName(ARM);
            ArmPistons.Clear();

            List<IMyTerminalBlock> termBlocks = new List<IMyTerminalBlock>();
            ArmGroup.GetBlocks(termBlocks);

            foreach (IMyTerminalBlock next in termBlocks)
            {
                IMyPistonBase piston = (IMyPistonBase)next;
                if (piston != null)
                    ArmPistons.Add(piston);
            }
        }

        /// Main Section

        public void TestMotion(IMyShipController control)
        {
            if (control != null)
            {
                YawBase.SetValueFloat("Velocity", Clamp(control.RotationIndicator.Y * YAW_DIR, -YAW_MAX, YAW_MAX));
                PitchBase.SetValueFloat("Velocity", Clamp(control.MoveIndicator.X * PITCH_DIR, -PITCH_MAX, PITCH_MAX));

                foreach (IMyPistonBase next in ArmPistons)
                    next.SetValueFloat("Velocity", -(control.MoveIndicator.Z * ARM_VELOCITY));
            }
        }

        #endregion
        #region Main


        public void Main1(string argument, UpdateType updateSource)
        {
            //DisplayInputValues(ArmControl);
            //TestMotion(ArmControl);

            AutoTrackTest();
            Echo("Working:");
            Echo("\nCurrentTarget: " + Sample.Entity.Name);
            Echo("\nCurrentKeyword: " + Sample.Keyword);
            Echo("\nCurrentType: " + Sample.EntityType);

            switch (argument)
            {
                case "TRACK":
                    TrackingOn = !TrackingOn;
                    break;

                case "UPDATE":
                    Sample.UpdateTargetParamaters();
                    break;
            }

        }

        #endregion

        #endregion

        #region DRONE

        // User Defs //

        const string OutChannel = "DRONE_HUB";
        const string SwarmChannel = "DRONE_SWARM";
        const string PanelName = "PANEL";
        const string ControlName = "CONTROL";
        const string PortName = "PORT";
        const string SensorName = "SENSOR";
        const string ShieldControlName = "[A] Shield Controller";

        const float MAX_DRIFT_MIN = 10f;
        const float MAX_DRIFT_MAX = 1000f;
        const float DOCK_PROXY = 15f;
        const float CHECK_PROXY = 1f;
        const float DRIFT_PROXY = 0.1f;
        const double GYRO_PROXY = 1.6;
        const float THRUST_SCALE = 100000f;
        const float COLL_THRUST_SCALE = 100f;
        const float GYRO_SCALE = 0.01f;
        const double RAD2DEG = 180 / Math.PI;
        const int EVENT_TIME = 20;
        const int EXP_TIME = 50;

        string InChannel;
        string DebugLog;
        int EventClock = 0;
        int ExpirationClock = 0;

        bool bConfigured = false;
        bool bShieldAvailable = false;
        bool bPortAvailable = false;
        bool bRCavailable = false;
        bool bRunning = false;
        bool bTargetGood = false;
        bool bResponsePending = false;

        bool bTrueFlight = false;
        bool bDocking = false;
        bool bDockingInitialized = false;
        bool bDockingProxy = false;

        FlightMode Mode = FlightMode.STANDBY;
        FlightMode OldMode;

        IMyTerminalBlock DroneShield;
        IMyBroadcastListener DroneEar;
        IMyBroadcastListener SwarmEar;
        IMyTextSurface DroneSurface;
        IMyTextSurface CockpitScreen;
        IMyRemoteControl DroneRC;
        IMyShipController DroneControl;
        IMyShipConnector DronePort;
        IMySensorBlock DroneSensor;

        List<IMySensorBlock> DroneSensors;

        // WIP
        IMyTurretControlBlock TurretControl;
        IMyTextPanel Panel;
        //

        IMyTerminalBlock HEAD;

        Vector3 TARGET;
        Vector3 DELTA_0;
        Vector3 DELTA_1;
        Vector3 Look;
        Vector3 HUB_DRIFT;

        bool SWITCH_0; // distinguish between old coord and docking coord
        bool SWITCH_1; // 

        double MaxDrift = 50f;
        float[] ThrustRatios = new float[6];
        float[] ThrustVectors = new float[6];
        double[] GyroVectors = new double[3];
        long[] RawMessage = new long[2];

        List<IMyThrust>[] ThrustGroups;
        List<GyroMask> Gyros;
        Sequence DockingSequence;

        public enum Trigger
        {
            DELTA_0,
            DELTA_1,
            EVENT
        }
        public enum DroneRequest
        {
            REGISTER,
            REASSIGN,
            FORM,
            DOCK,
            RELEASE
        }
        public enum FlightMode
        {
            STANDBY,
            NAV,
            MIMIC,
            RESUPPLY,
            SKYNET
        }
        public enum GyroAction
        {
            PITCH,
            YAW,
            ROLL
        }

        public struct ActionMask
        {
            public GyroAction Action;
            public int Sign;

            public ActionMask(GyroAction action, int sign = 1)
            {
                Action = action;
                Sign = sign;
            }
        }
        public struct Operation
        {
            public Trigger Trigger;
            public FlightMode Mode;

            public Operation(Trigger trigger, FlightMode mode)
            {
                Trigger = trigger;
                Mode = mode;
            }
        }
        public class Sequence
        {
            public int CurrentIndex;
            public Operation[] Operations;

            public Sequence(Operation[] operations)
            {
                Operations = operations;
                CurrentIndex = 0;
            }
        }
        public class GyroMask
        {
            public IMyGyro Gyro;
            public ActionMask[] RCactions;    //  0 = pitch, 1 = yaw, 2 = roll 
            public ActionMask[] DockActions;

            public GyroMask(IMyGyro gyro)
            {
                Gyro = gyro;
                RCactions = new ActionMask[3];
                DockActions = new ActionMask[3];
            }
        }

        void DockingSequenceBuilder()
        {
            Operation[] operations = new Operation[4];

            operations[0] = new Operation(Trigger.DELTA_0, FlightMode.NAV);      // Head towards docking port
            operations[1] = new Operation(Trigger.DELTA_1, FlightMode.MIMIC);    // Approach docking port and connect
            operations[2] = new Operation(Trigger.EVENT, FlightMode.RESUPPLY);   // Resupply and disconnect
            operations[3] = new Operation(Trigger.DELTA_0, FlightMode.MIMIC);    // Clear away from docking port

            DockingSequence = new Sequence(operations);
        }

        GyroMask SetupGyroMask(IMyGyro gyro)
        {
            GyroMask newMask = new GyroMask(gyro);

            newMask.RCactions = SetupGyroActions(DroneControl, gyro);
            if (bPortAvailable)
                newMask.DockActions = SetupGyroActions(DronePort, gyro);
            return newMask;
        }
        int[] GenerateAxisData(IMyTerminalBlock myBoop)
        {
            // RUB (right, up, back)
            Vector3 pitchVector = -Base6Directions.Directions[(int)myBoop.Orientation.Left]; // take negative, need right vector
            Vector3 yawVector = Base6Directions.Directions[(int)myBoop.Orientation.Up];
            Vector3 rollVector = -Base6Directions.Directions[(int)myBoop.Orientation.Forward]; // take negative, need back vector

            int pitchAxis = 0;
            int yawAxis = 0;
            int rollAxis = 0;
            int pitchSign = 0;
            int yawSign = 0;
            int rollSign = 0;

            for (int i = 0; i < 3; i++)
            {
                if (pitchVector.GetDim(i) != 0)
                {
                    pitchAxis = i;
                    pitchSign = (int)pitchVector.GetDim(i);
                }
                if (yawVector.GetDim(i) != 0)
                {
                    yawAxis = i;
                    yawSign = (int)yawVector.GetDim(i);
                }
                if (rollVector.GetDim(i) != 0)
                {
                    rollAxis = i;
                    rollSign = (int)rollVector.GetDim(i);
                }
            }

            return new int[] { pitchAxis, yawAxis, rollAxis, pitchSign, yawSign, rollSign };
        }
        ActionMask[] SetupGyroActions(IMyTerminalBlock parent, IMyTerminalBlock child)
        {
            int[] childSet = GenerateAxisData(child);
            int[] parentSet = GenerateAxisData(parent);

            ActionMask[] newActions = new ActionMask[3];

            for (int i = 0; i < 3; i++)
            {
                for (int j = 0; j < 3; j++)
                {
                    if (childSet[j] == parentSet[i])
                    {
                        newActions[i].Action = (GyroAction)j;
                        newActions[i].Sign = parentSet[i + 3] * childSet[j + 3];
                    }
                }
            }

            return newActions;
        }

        void SetupRemoteControl()
        {
            DroneControl = (IMyShipController)GridTerminalSystem.GetBlockWithName(ControlName);
            if (DroneControl is IMyCockpit)
            {
                CockpitScreen = ((IMyCockpit)DroneControl).GetSurface(0);
                CockpitScreen.ContentType = ContentType.TEXT_AND_IMAGE;
                CockpitScreen.WriteText("");
            }
                
            if (!(DroneControl is IMyRemoteControl))
                return;

            bRCavailable = true;
            DroneRC = (IMyRemoteControl)DroneControl;
            DroneRC.FlightMode = Sandbox.ModAPI.Ingame.FlightMode.OneWay;
            DroneRC.SetDockingMode(true);
            DroneRC.SetCollisionAvoidance(true);
            DroneRC.SetAutoPilotEnabled(false);
            DroneRC.ClearWaypoints();
        }
        void SetupSensor(IMySensorBlock sensor)
        {
            if (sensor == null)
                return;

            sensor.DetectOwner = true;
            sensor.DetectFriendly = true;
            sensor.DetectEnemy = true;
            sensor.DetectNeutral = true;

            sensor.DetectStations = true;
            sensor.DetectLargeShips = true;
            sensor.DetectSmallShips = true;
            sensor.DetectSubgrids = true;
            sensor.DetectAsteroids = true;
            sensor.DetectPlayers = false;

            sensor.BackExtend = 50;
            sensor.BottomExtend = 50;
            sensor.FrontExtend = 50;
            sensor.LeftExtend = 50;
            sensor.RightExtend = 50;
            sensor.TopExtend = 50;
        }
        void PopulateGyros()
        {
            Gyros = new List<GyroMask>();
            List<IMyGyro> gyros = new List<IMyGyro>();
            GridTerminalSystem.GetBlocksOfType(gyros);

            foreach (IMyGyro gyro in gyros)
                Gyros.Add(SetupGyroMask(gyro));

            Echo("gyros populated");
        }
        void PopulateThrusters()
        {
            ThrustGroups = new List<IMyThrust>[6];

            List<float> thrustTotals = new List<float>();
            List<IMyThrust> thrusters = new List<IMyThrust>();
            GridTerminalSystem.GetBlocksOfType(thrusters);

            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            for (int i = 0; i < 6; i++)
            {
                ThrustGroups[i] = new List<IMyThrust>();
                float thrustTotal = 0;

                foreach (IMyThrust thrust in thrusters)
                {
                    if (thrust.Orientation.Forward != (Base6Directions.Direction)i)
                        continue;

                    ThrustGroups[i].Add(thrust);
                    thrustTotal += thrust.MaxThrust;
                }

                thrustTotals.Add(thrustTotal);
            }

            int weakestIndex = 0;

            for (int i = 1; i < 6; i++)
            {
                if (thrustTotals[i] < thrustTotals[weakestIndex])
                    weakestIndex = i;
            }

            float criticalAccel = thrustTotals[weakestIndex] / DroneControl.CalculateShipMass().TotalMass;
            float stopDistance = DroneSensor == null ? 10 : DroneSensor.MaxRange - (float)DroneControl.WorldAABB.HalfExtents.Z; // Subtract half the 'length' of the ship from the sensor radius
            MaxDrift = Math.Sqrt((2 * stopDistance)/criticalAccel) * criticalAccel;

            for (int i = 0; i < 6; i++)
            {
                ThrustRatios[i] = thrustTotals[weakestIndex] / thrustTotals[i];
            }

            Echo("thrusters populated");
        }

        void Toggle(bool on)
        {
            bRunning = on;

            if (!bRunning && Mode == FlightMode.MIMIC)
            {
                ApplyGyros(true);
                ApplyThrust(true);
            }

            if (bRunning && !bResponsePending)
                Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
        }
        void SwarmResponse(MyIGCMessage message)
        {
            try
            {
                int data = (int)message.Data;

                switch (data)
                {
                    case 0:
                        Toggle(false);
                        break;

                    case 1:
                        Toggle(true);
                        break;

                    case 2:
                        SwitchModes(FlightMode.STANDBY);
                        break;

                    case 3:
                        SwitchModes(FlightMode.NAV);
                        break;

                    case 4:
                        SwitchModes(FlightMode.MIMIC);
                        break;

                    case 5:
                        AdjustShield(false);
                        break;

                    case 6:
                        AdjustShield(true);
                        break;

                    case 7:
                        AdjustSpeed(false);
                        break;

                    case 8:
                        AdjustSpeed(true);
                        break;

                    case 9:
                        bTrueFlight = true;
                        break;

                    case 10:
                        bTrueFlight = false;
                        break;

                    default:
                        DroneSurface.WriteText("Out of range!");
                        break;
                }
            }
            catch
            {
                DroneSurface.WriteText("Failed to interpret message. Not an integer!");
            }
        }
        void Request(DroneRequest request)
        {
            DebugLog += "Requesting...\n";

            bResponsePending = true;

            RawMessage[1] = (long)request;

            var data = ImmutableArray.Create(RawMessage);

            IGC.SendBroadcastMessage(OutChannel, data);
        }
        bool Recieve(MyIGCMessage message)
        {
            if (message.Data is ImmutableArray<double>)
            {
                ImmutableArray<double> data = (ImmutableArray<double>)message.Data;

                DELTA_0.X = (float)data.ItemRef(0); DELTA_0.Y = (float)data.ItemRef(1); DELTA_0.Z = (float)data.ItemRef(2);
                DELTA_1.X = (float)data.ItemRef(3); DELTA_1.Y = (float)data.ItemRef(4); DELTA_1.Z = (float)data.ItemRef(5);
                HUB_DRIFT.X = (float)data.ItemRef(6); HUB_DRIFT.Y = (float)data.ItemRef(7); HUB_DRIFT.Z = (float)data.ItemRef(8);

                try
                {
                    SWITCH_0 = ((int)data.ItemRef(9)) < 0 ? false : true;
                }
                catch
                {
                    SWITCH_0 = false;
                }

                if (bDocking)
                {
                    Look = DELTA_1 - DELTA_0;

                    if (!bDockingInitialized &&
                        SWITCH_0 == true)
                    {
                        bDockingInitialized = true;
                    }
                }

                else
                    Look = DELTA_0 - DELTA_1;

                DebugLog += "Vector data recieved!\n";
                return true;
            }

            if (message.Data is string)
            {
                // put something here? I dunno fuck...
                DebugLog += $"Message recieved: {message.Data}\n";
                return false;
            }

            DebugLog += "Couldn't translate!\n";

            return false;
        }

        Vector3 GenerateCollisionAvoidanceVector(IMyTerminalBlock head, bool kill = false)
        {
            Vector3 output = new Vector3();

            if (kill == true)
                return output;

            if (DroneSensor == null &&
                DroneSensors.Count == 0)
                return output;

            List<MyDetectedEntityInfo> nextSet = new List<MyDetectedEntityInfo>();
            List<MyDetectedEntityInfo> detectedEntities = new List<MyDetectedEntityInfo>();
            if (DroneSensor != null)
                DroneSensor.DetectedEntities(detectedEntities);

            foreach(IMySensorBlock sensor in DroneSensors)
            {
                sensor.DetectedEntities(nextSet);

                foreach(MyDetectedEntityInfo info in detectedEntities)
                    nextSet.Remove(info);

                detectedEntities.AddRange(nextSet);
            }

            if (detectedEntities.Count == 0)
                return output;

            int count = 0;

            foreach (MyDetectedEntityInfo entity in detectedEntities)
            {
                DebugLog += $"detecting:{count}\n";
                count++;
                Vector3 delta;
                float distance;
                if (entity.HitPosition != null)
                {
                    delta = head.GetPosition() - entity.HitPosition.Value;
                    distance = Vector3.Distance(Vector3.Zero, delta);
                }
                else
                {
                    delta = head.GetPosition() - entity.Position;
                    distance = Vector3.Distance(Vector3.Zero, delta);

                    double maxExtents = entity.BoundingBox.HalfExtents.X > entity.BoundingBox.HalfExtents.Y ? entity.BoundingBox.HalfExtents.X : entity.BoundingBox.HalfExtents.Y;
                    maxExtents = entity.BoundingBox.HalfExtents.Z > maxExtents ? entity.BoundingBox.HalfExtents.Z : maxExtents;
                    distance -= (float)maxExtents;
                }

                float magnitude = COLL_THRUST_SCALE / (distance * distance);
                Vector3 normalDelta = TransformVectorRelative(head.CubeGrid.WorldMatrix, delta) * magnitude;
                output += normalDelta;
            }

            return output;
        }
        Vector3 TransformVectorRelative(MatrixD S, Vector3 D) // S = sourceBearing, D = WorldVectorDelta
        {
            /* X,Y,Z = Normalized Vector Unit Coefficients
             * x,y,z = Delta Target Vector (raw World GPS)
             * a,b,c = Normalized X vector components (relative x,y,z)
             * d,e,f = Normalized Y ''
             * g,h,i = Normalized Z ''
             * 
             * Keen Implementation:
             * Row 1: Right.x , Right.y , Right.z
             * Row 2: Up.x    , Up.y    , Up.z
             * Row 3: Back.x  , Back.y  , Back.z
            */

            Vector3D NV = new Vector3D(); // new NormalizedVector

            // Z = (d(bz - cy) + e(cx - az) + f(ay - bx)) / (d(bi - ch) + e(cg - ai) + f(ah - bg))

            // Z = (d     * ((b     * z)   - (c     * y))   + e     * ((c     * x)   - (a     * z))   + f     * ((a     * y)   - (b     * x)))   / (d     * ((b     * i)     - (c     * h))     + e     * ((c     * g)     - (a     * i))     + f     * ((a     * h)     - (b     * g)))
            NV.Z = (S.M21 * ((S.M12 * D.Z) - (S.M13 * D.Y)) + S.M22 * ((S.M13 * D.X) - (S.M11 * D.Z)) + S.M23 * ((S.M11 * D.Y) - (S.M12 * D.X))) / (S.M21 * ((S.M12 * S.M33) - (S.M13 * S.M32)) + S.M22 * ((S.M13 * S.M31) - (S.M11 * S.M33)) + S.M23 * ((S.M11 * S.M32) - (S.M12 * S.M31)));


            // Y = (Z(hc - ib) + zb - yc) / (fb - ec)
            // Y = (Z(gb - ha) + ya - xb) / (ea - db)

            // Y = (Z    * ((h     * c)     - (i     * b))     + (z   * b)     - (y   * c))     / ((f     * b)     - (e     * c))
            NV.Y = (NV.Z * ((S.M32 * S.M13) - (S.M33 * S.M12)) + (D.Z * S.M12) - (D.Y * S.M13)) / ((S.M23 * S.M12) - (S.M22 * S.M13));

            // X = (x - (Yd + Zg)) / a
            // X = (y - (Ye + Zh)) / b
            // X = (z - (Yf + Zi)) / c

            // X = (x   - ((Y    * d)     + (Z    * g)))     / a
            NV.X = (D.X - ((NV.Y * S.M21) + (NV.Z * S.M31))) / S.M11;

            return NV;
        }
        Vector3 VelocityCapVector(Vector3 vector, double cap)
        {
            double mag = Vector3.Distance(vector, Vector3.Zero);
            if (mag == 0)
                return Vector3.Zero;
            Vector3 newVector = Vector3.Normalize(vector * 2);
            mag = mag > cap ? cap : mag;
            newVector *= (float)mag;
            return newVector;
        }

        void GenerateThrustVectors(IMyTerminalBlock head, Vector3 delta, bool kill = false)
        {
            /*  0 - Forward
             *  1 - Backward
             *  2 - Left
             *  3 - Right
             *  4 - Up
             *  5 - Down
             */

            if (kill)
            {
                for (int i = 0; i < 6; i++)
                    ThrustVectors[i] = 0;
                return;
            }

            float distance = Vector3.Distance(delta, head.GetPosition());
            Vector3 targetDelta = delta - head.GetPosition();
            Vector3 targetTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, targetDelta);
            Vector3 hubDriftTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, HUB_DRIFT);
            targetTrans += hubDriftTrans;

            Vector3 driftTrans = TransformVectorRelative(head.CubeGrid.WorldMatrix, DroneControl.GetShipVelocities().LinearVelocity);
            Vector3 collAvoid = GenerateCollisionAvoidanceVector(head, bDockingProxy);

            float collMagRaw = Vector3.Distance(collAvoid, Vector3.Zero);
            collAvoid = VelocityCapVector(collAvoid, MaxDrift);

            Vector3 thrustVector = Vector3.Normalize(targetTrans);
            float combinedMag = (distance) / (1 + collMagRaw);
            thrustVector *= combinedMag;
            thrustVector = VelocityCapVector(thrustVector, MaxDrift);
            thrustVector += collAvoid - driftTrans;
            thrustVector *= THRUST_SCALE;

            DebugLog += $"Maxrift: {MaxDrift}\n";
            DebugLog += $"distance: {distance}\n";
            DebugLog += $"targetDelta: {targetDelta}\n";
            DebugLog += $"collAvoid: {collAvoid}\n";
            DebugLog += $"targetNormal: {targetTrans}\n";
            DebugLog += $"driftNormal: {driftTrans}\n";
            DebugLog += $"netNormal: {thrustVector}\n";

            ThrustVectors[0] = thrustVector.Z * ThrustRatios[0];
            ThrustVectors[1] = -thrustVector.Z * ThrustRatios[1];
            ThrustVectors[2] = thrustVector.X * ThrustRatios[2];
            ThrustVectors[3] = -thrustVector.X * ThrustRatios[3];
            ThrustVectors[4] = -thrustVector.Y * ThrustRatios[4];
            ThrustVectors[5] = thrustVector.Y * ThrustRatios[5];
        }
        void UpdateBlockMeta()
        {
            if (bDocking)
            {
                HEAD = DronePort;
                Trigger trig = DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger;
                TARGET = (trig == Trigger.DELTA_0) ? DELTA_0 : DELTA_1;
            }
            else
            {
                HEAD = DroneControl;
                TARGET = DELTA_0;
            }

            DebugLog += $"Target: {TARGET}\n";
        }
        void GenerateGyroVectors(bool kill = false)
        {
            if (kill)
            {
                GyroVectors[0] = 0;
                GyroVectors[1] = 0;
                GyroVectors[2] = 0;
                return;
            }

            IMyTerminalBlock head = (bDocking) ? (IMyTerminalBlock)DronePort : DroneControl;

            ////////////////

            Vector3 lookVector;

            if (bTrueFlight)
                lookVector = DroneControl.GetShipVelocities().LinearVelocity;
            else
                lookVector = Look;

            Vector3 normalLook = TransformVectorRelative(head.WorldMatrix, lookVector);

            double targetYaw = Math.Atan2(normalLook.X, -normalLook.Z);
            double targetPitch = Math.Atan2(normalLook.Y, -normalLook.Z);
            //double targetRoll

            targetYaw = (targetYaw > Math.PI) ? -((2 * Math.PI) - targetYaw) : targetYaw;
            targetPitch = (targetPitch > Math.PI) ? -((2 * Math.PI) - targetPitch) : targetPitch;

            targetYaw *= RAD2DEG;
            targetPitch *= RAD2DEG;

            targetYaw *= GYRO_SCALE;
            targetPitch *= GYRO_SCALE;

            /*
            DebugLog += $"Target: {Target}\n";
            DebugLog += $"Look: {Look}\n";
            DebugLog += $"Normal: {normalLook}\n";
            DebugLog += $"Yaw: {targetYaw}\n";
            DebugLog += $"Pitch: {targetPitch}\n";
            */

            for (int i = 0; i < 3; i++)
                DebugLog += $"{Gyros[0].RCactions[i].Action} : {Gyros[0].RCactions[i].Sign}\n";

            GyroVectors[0] = -targetPitch;
            GyroVectors[1] = targetYaw;
            GyroVectors[2] = 0; // Sans rrrroller pas exactement ci vous plais

        }
        void ApplyGyros(bool kill = false)
        {
            GenerateGyroVectors(kill);

            for (int i = 0; i < 3; i++)
            {
                foreach (GyroMask gyro in Gyros)
                {
                    ApplyGyroAction(gyro, i, GyroVectors[i]);
                }
            }
        }
        void ApplyGyroAction(GyroMask mask, int action, double value)
        {
            ActionMask[] actMask = (bDocking) ? mask.DockActions : mask.RCactions;

            switch (actMask[action].Action)
            {
                case GyroAction.YAW:
                    mask.Gyro.Yaw = (float)value * actMask[action].Sign;
                    break;

                case GyroAction.PITCH:
                    mask.Gyro.Pitch = (float)value * actMask[action].Sign;
                    break;

                case GyroAction.ROLL:
                    mask.Gyro.Roll = (float)value * actMask[action].Sign;
                    break;
            }
        }
        void ApplyThrust(bool kill = false)
        {
            DebugLog += "Applying Thrust...\n" +
                $"kill : {kill}\n";

            UpdateBlockMeta();
            GenerateThrustVectors(HEAD, TARGET, kill);

            for (int i = 0; i < 6; i++)
            {
                foreach (IMyThrust thrust in ThrustGroups[i])
                {
                    thrust.ThrustOverride = ThrustVectors[i];
                }
            }
        }

        void DockingUpdate()
        {
            if (!bDocking || !bDockingInitialized)
                return;

            if (!bDockingProxy)
                bDockingProxy = ProxyCheckDocking();

            switch (DockingSequence.Operations[DockingSequence.CurrentIndex].Trigger)
            {
                case Trigger.DELTA_0:

                    if (!ProxyCheckLiteral())
                        break;

                    if (DockingSequence.CurrentIndex == 0)
                    {
                        DronePort.Enabled = true;
                        DockingSequence.CurrentIndex++;
                    }

                    if (DockingSequence.CurrentIndex == 3)
                    {
                        Mode = OldMode;
                        DronePort.Enabled = false;
                        bDocking = false;
                        bDockingInitialized = false;
                        bDockingProxy = false;
                        DockingSequence.CurrentIndex = 0;
                        Request(DroneRequest.RELEASE);
                        Request(DroneRequest.FORM);
                    }
                    break;

                case Trigger.DELTA_1:

                    if (DronePort.Status == MyShipConnectorStatus.Connectable)
                    {
                        DronePort.Connect();
                        SwitchModes(FlightMode.RESUPPLY);
                        DockingSequence.CurrentIndex++;
                    }

                    break;

                case Trigger.EVENT:

                    EventClock++;
                    DebugLog += $"EventClock: {EventClock}\n";

                    if (EventClock >= EVENT_TIME)
                    {
                        DronePort.Disconnect();
                        SwitchModes(FlightMode.MIMIC);
                        EventClock = 0;
                        DockingSequence.CurrentIndex++;
                    }
                    break;
            }

            DebugLog += $"DockingSequenceIndex: {DockingSequence.CurrentIndex}\n";
        }
        void NavMode()
        {
            if (!bRCavailable ||
                Mode != FlightMode.NAV ||
                !ProxyCheckNav())
                return;

            if (bTargetGood && bRCavailable)
                Move();
            else
                Request(DroneRequest.FORM);
        }
        void MimicMode()
        {
            if (Mode != FlightMode.MIMIC)
                return;

            //Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
            if (!bTargetGood)
            {
                ApplyThrust(true);
                ApplyGyros(true);
            }
            else
            {
                ApplyThrust();
                ApplyGyros();
            }
        }
        void Idle()
        {
            //DroneSurface.WriteText("FAILED TO RECIEVE TARGET!", false);
        }
        void Move()
        {
            DebugLog += $"Moving... \n";
            DroneRC.ClearWaypoints();
            DroneRC.AddWaypoint(DELTA_0, "TARGET");
            DroneRC.SetAutoPilotEnabled(true);
            bTargetGood = false;
            //bDockingInitialized = bDocking;
        }
        bool ProxyCheckDocking()
        {
            DebugLog += $"DockProxy: {Vector3.Distance(HEAD.GetPosition(), DELTA_0)} | {DOCK_PROXY} | {Vector3.Distance(HEAD.GetPosition(), DELTA_0) < DOCK_PROXY}\n";
            return Vector3.Distance(HEAD.GetPosition(), DELTA_0) < DOCK_PROXY;
        }
        bool ProxyCheckNav()
        {
            DebugLog += "ProxyCheck... \n";
            return !DroneRC.IsAutoPilotEnabled;
        }
        bool ProxyCheckLiteral()
        {
            DebugLog += $"Distance: {Vector3.Distance(TARGET, HEAD.GetPosition())} | {CHECK_PROXY} | {Vector3.Distance(TARGET, DroneControl.GetPosition()) < CHECK_PROXY}\n" +
                $"Drift: {Vector3.Distance(DroneControl.GetShipVelocities().LinearVelocity, HUB_DRIFT)} | {DRIFT_PROXY} | {Vector3.Distance(DroneControl.GetShipVelocities().LinearVelocity, HUB_DRIFT) < DRIFT_PROXY}\n" +
                $"Gyros: {Math.Abs(GyroVectors[0] + GyroVectors[1] + GyroVectors[2])} | {GYRO_PROXY} | {Math.Abs(GyroVectors[0] + GyroVectors[1] + GyroVectors[2]) < GYRO_PROXY}\n";

            return Vector3.Distance(TARGET, HEAD.GetPosition()) < CHECK_PROXY &&
                Vector3.Distance(DroneControl.GetShipVelocities().LinearVelocity, HUB_DRIFT) < DRIFT_PROXY &&
                Math.Abs(GyroVectors[0] + GyroVectors[1] + GyroVectors[2]) < GYRO_PROXY;
        }

        void ToggleGyros(bool toggle = false)
        {
            foreach (GyroMask gyro in Gyros)
            {
                if (!toggle)
                {
                    gyro.Gyro.Yaw = 0;
                    gyro.Gyro.Pitch = 0;
                    gyro.Gyro.Roll = 0;
                }

                gyro.Gyro.GyroOverride = toggle;
            }
        }
        void SwitchModes(FlightMode mode)
        {
            Mode = mode;

            if (Mode == FlightMode.MIMIC)
                ToggleGyros(true);

            if (Mode != FlightMode.MIMIC)
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            if (Mode != FlightMode.NAV &&
                bRCavailable)
                DroneRC.SetAutoPilotEnabled(false);
        }
        void SwitchModes()
        {
            Mode = (Mode == FlightMode.MIMIC) ? 0 : Mode + 1;

            if (Mode == FlightMode.MIMIC)
                ToggleGyros(true);

            if (Mode != FlightMode.MIMIC)
            {
                ApplyThrust(true);
                ToggleGyros();
            }

            if (Mode != FlightMode.NAV && DroneRC != null)
                DroneRC.SetAutoPilotEnabled(false);
        }
        void AdjustSpeed(bool faster)
        {
            MaxDrift = MaxDrift < MAX_DRIFT_MAX && faster ? MaxDrift + 10 : MaxDrift;
            MaxDrift = MaxDrift > MAX_DRIFT_MIN && !faster ? MaxDrift - 10 : MaxDrift;
        }
        void AdjustShield(bool bigger)
        {
            if (!bShieldAvailable)
                return;

            try
            {
                float newVal = DroneShield.GetValueFloat("DS-CFit");
                newVal = newVal < 22 && bigger ? newVal + 1 : newVal;
                newVal = newVal > 1 && !bigger ? newVal - 1 : newVal;
                DroneShield.SetValueFloat("DS-CFit", newVal);
            }
            catch
            {
                return;
            }
        }

        void InitiateDockingSequence()
        {
            if (!bPortAvailable)
                return;

            OldMode = Mode;
            bDocking = true;
            bDockingInitialized = false;
            bTargetGood = false;
            DronePort.Enabled = false;
            DockingSequence.CurrentIndex = 0;
            SwitchModes(FlightMode.MIMIC);
            Request(DroneRequest.DOCK);
        }


        public Program()
        {
            DroneShield = GridTerminalSystem.GetBlockWithName(ShieldControlName);
            bShieldAvailable = DroneShield != null;
            string shield = bShieldAvailable ? "ShieldCalibrated" : "ShieldMissing";
            Echo(shield);

            DroneSensors = new List<IMySensorBlock>();
            IMyBlockGroup group = GridTerminalSystem.GetBlockGroupWithName(SensorName);
            if (group != null)
            {
                List<IMyTerminalBlock> blocks = new List<IMyTerminalBlock>();
                group.GetBlocks(blocks);
                foreach (IMyTerminalBlock block in blocks)
                {
                    if (block is IMySensorBlock)
                    {
                        SetupSensor((IMySensorBlock)block);
                        DroneSensors.Add((IMySensorBlock)block);
                    }      
                }       
            }
            DroneSensor = (IMySensorBlock)GridTerminalSystem.GetBlockWithName(SensorName);
            SetupSensor(DroneSensor);

            string echo = DroneSensor != null || DroneSensors != null || DroneSensors.Count > 0 ? "SensorCalibrated" : "SensorMissing";
            Echo(echo);

            try
            {
                Panel = (IMyTextPanel)GridTerminalSystem.GetBlockWithName(PanelName);
                DronePort = (IMyShipConnector)GridTerminalSystem.GetBlockWithName(PortName);
                bPortAvailable = DronePort != null;
                if (bPortAvailable)
                    DronePort.Enabled = false;

                DroneSurface = Me.GetSurface(0);
                DroneSurface.ContentType = ContentType.TEXT_AND_IMAGE;
                DroneSurface.WriteText("", false);
                DebugLog = string.Empty;

                Me.CustomName = "DRONE_PB";
                RawMessage[0] = Me.EntityId;
                Me.CubeGrid.CustomName = $"DRONE_{Me.EntityId}";
                InChannel = $"DRONE_{Me.EntityId}";
                SwarmEar = IGC.RegisterBroadcastListener(SwarmChannel);
                DroneEar = IGC.RegisterBroadcastListener(InChannel);
                DroneEar.SetMessageCallback();

                DockingSequenceBuilder();
                SetupRemoteControl();
                PopulateThrusters();
                PopulateGyros();

                SwitchModes(FlightMode.STANDBY);

                Runtime.UpdateFrequency = UpdateFrequency.Update10;

                Load();

                Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);

                bConfigured = true;
            }
            catch
            {
                bConfigured = false;
            }
        }
        public void Main(string argument, UpdateType updateSource)
        {
            DroneSurface.WriteText(DebugLog);
            if (CockpitScreen != null)
                CockpitScreen.WriteText(DebugLog);
            DebugLog = string.Empty; // DO NOT REMOVE THIS!!!!
            //DroneSurface.WriteText($"MaxDrift: {MaxDrift}");

            switch (argument)
            {
                case "TOGGLE":
                    Toggle(!bRunning);
                    break;

                case "TEST":
                    InitiateDockingSequence();
                    break;

                case "MODE":
                    SwitchModes();
                    break;

                case "CLEAR":
                    Toggle(false);
                    SwitchModes(FlightMode.STANDBY);
                    break;
            }

            while (SwarmEar.HasPendingMessage)
                SwarmResponse(SwarmEar.AcceptMessage());

            while (DroneEar.HasPendingMessage)
            {
                bTargetGood = Recieve(DroneEar.AcceptMessage());
                Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
                //bResponsePending = true;
            }

            if (bResponsePending)
            {
                ExpirationClock++;
                if (ExpirationClock == EXP_TIME)
                {
                    ExpirationClock = 0;
                    Request((bDocking) ? DroneRequest.DOCK : DroneRequest.FORM);
                }
            }

            if (!bRunning || !bConfigured)
                return;

            MimicMode();
            DockingUpdate();
            NavMode();
        }

        public void Save()
        {
            Storage = $"{bRunning}:{(int)Mode}:{bResponsePending}";
        }
        public void Load()
        {
            try
            {
                string[] raw = Storage.Split(':');
                bRunning = bool.Parse(raw[0]);
                Mode = (FlightMode)int.Parse(raw[1]);
                bResponsePending = bool.Parse(raw[2]);
                SwitchModes(Mode);
                Echo("Loaded");
            }
            catch
            {
                Echo("Nothing to load");
            }
        }

        #endregion


    }
}

