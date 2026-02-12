using System.Net.Http;
using System.Text;
using System.Text.Json;

namespace RobotCLI;

/// <summary>
/// Command-line interface for controlling the Robot Simulator.
/// Sends HTTP requests to the embedded API server (http://localhost:8085/api/).
/// </summary>
class Program
{
    private static readonly HttpClient _http = new();
    private const string BASE_URL = "http://localhost:8085/api";

    static async Task<int> Main(string[] args)
    {
        if (args.Length == 0)
        {
            PrintHelp();
            return 1;
        }

        var command = args[0].ToLower();

        try
        {
            string result;

            if (command == "status")
                result = await Get("/status");
            else if (command == "joints")
                result = await Get("/joints");
            else if (command == "move")
                result = await PostJoints(args);
            else if (command == "home")
                result = await Post("/home");
            else if (command == "teach")
                result = await Post("/teach");
            else if (command == "run")
                result = await Post("/run");
            else if (command == "estop")
                result = await Post("/estop");
            else if (command == "program")
                result = await Get("/program");
            else if (command == "clear")
                result = await Delete("/program");
            else if (command == "connect" && args.Length >= 2 && args[1].ToLower() == "ros")
                result = await Post("/connect/ros");
            else if (command == "help" || command == "--help" || command == "-h")
            {
                PrintHelp();
                return 0;
            }
            else
                throw new ArgumentException($"Unknown command: {command}");

            // Pretty print JSON
            try
            {
                var doc = JsonDocument.Parse(result);
                Console.WriteLine(JsonSerializer.Serialize(doc, new JsonSerializerOptions { WriteIndented = true }));
            }
            catch
            {
                Console.WriteLine(result);
            }

            return 0;
        }
        catch (HttpRequestException)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.Error.WriteLine("ERROR: Cannot connect to Robot Simulator.");
            Console.Error.WriteLine("       Make sure RobotSimulator.exe is running.");
            Console.Error.WriteLine($"       Expected API at: {BASE_URL}");
            Console.ResetColor();
            return 2;
        }
        catch (Exception ex)
        {
            Console.ForegroundColor = ConsoleColor.Red;
            Console.Error.WriteLine($"ERROR: {ex.Message}");
            Console.ResetColor();
            return 1;
        }
    }

    static async Task<string> Get(string path)
    {
        var resp = await _http.GetAsync(BASE_URL + path);
        resp.EnsureSuccessStatusCode();
        return await resp.Content.ReadAsStringAsync();
    }

    static async Task<string> Post(string path, string? body = null)
    {
        var content = body != null
            ? new StringContent(body, Encoding.UTF8, "application/json")
            : new StringContent("{}", Encoding.UTF8, "application/json");
        var resp = await _http.PostAsync(BASE_URL + path, content);
        resp.EnsureSuccessStatusCode();
        return await resp.Content.ReadAsStringAsync();
    }

    static async Task<string> Delete(string path)
    {
        var resp = await _http.DeleteAsync(BASE_URL + path);
        resp.EnsureSuccessStatusCode();
        return await resp.Content.ReadAsStringAsync();
    }

    static async Task<string> PostJoints(string[] args)
    {
        if (args.Length < 7)
            throw new ArgumentException("Usage: RobotCLI move <j1> <j2> <j3> <j4> <j5> <j6> (degrees)");

        var body = JsonSerializer.Serialize(new
        {
            j1 = double.Parse(args[1]),
            j2 = double.Parse(args[2]),
            j3 = double.Parse(args[3]),
            j4 = double.Parse(args[4]),
            j5 = double.Parse(args[5]),
            j6 = double.Parse(args[6])
        });

        return await Post("/joints", body);
    }

    static void PrintHelp()
    {
        Console.WriteLine(@"
  ROBOT SIMULATOR CLI v2.0 - AI-Controllable Robot Interface

USAGE:
  RobotCLI <command> [arguments]

COMMANDS:
  status              Get current robot state (joints, TCP, connection)
  joints              Get current joint angles only
  move <j1>..<j6>     Move joints to specified angles (degrees)
  home                Move all joints to 0 degrees
  teach               Save current position as waypoint
  run                 Execute loaded program
  estop               Emergency stop
  program             List all waypoints
  clear               Clear all waypoints
  connect ros         Connect to ROS bridge

EXAMPLES:
  RobotCLI status
  RobotCLI move 45 30 0 0 0 0
  RobotCLI teach
  RobotCLI run

API:
  The simulator exposes a REST API at http://localhost:8085/api/
  You can also use curl:
    curl http://localhost:8085/api/status
    curl -X POST http://localhost:8085/api/joints -d ""{""""j1"""":45,""""j2"""":30}""
");
    }
}
