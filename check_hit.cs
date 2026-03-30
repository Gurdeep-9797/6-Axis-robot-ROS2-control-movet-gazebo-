using System;
using System.Reflection;
using HelixToolkit.Wpf;

class Program
{
    static void Main()
    {
        var type = typeof(PointHitResult);
        foreach (var prop in type.GetProperties())
        {
            Console.WriteLine(prop.Name + " : " + prop.PropertyType.Name);
        }
    }
}
