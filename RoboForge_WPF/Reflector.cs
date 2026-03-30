using System;
using System.IO;
using System.Reflection;
using HelixToolkit.Wpf;

namespace RoboForge_WPF
{
    public class Reflector
    {
        public static void Check()
        {
            var type = typeof(PointHitResult);
            using var w = new StreamWriter("hit_props.txt");
            foreach (var prop in type.GetProperties())
            {
                w.WriteLine(prop.Name + " : " + prop.PropertyType.Name);
            }
        }
    }
}
