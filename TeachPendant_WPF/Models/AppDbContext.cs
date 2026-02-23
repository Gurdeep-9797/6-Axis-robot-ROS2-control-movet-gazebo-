using System;
using System.Collections.Generic;
using Microsoft.EntityFrameworkCore;

namespace TeachPendant_WPF.Models
{
    public class AppDbContext : DbContext
    {
        public DbSet<ProgramEntity> Programs { get; set; }
        public DbSet<NodeEntity> Nodes { get; set; }
        public DbSet<RobotConfigEntity> RobotConfigs { get; set; }

        public AppDbContext()
        {
            // Create database file in local AppData folder
            Database.EnsureCreated();
        }

        protected override void OnConfiguring(DbContextOptionsBuilder optionsBuilder)
        {
            string dbPath = System.IO.Path.Combine(
                Environment.GetFolderPath(Environment.SpecialFolder.LocalApplicationData),
                "FR_TeachPendant",
                "robot_database.sqlite");
                
            System.IO.Directory.CreateDirectory(System.IO.Path.GetDirectoryName(dbPath)!);
            optionsBuilder.UseSqlite($"Data Source={dbPath}");
        }

        protected override void OnModelCreating(ModelBuilder modelBuilder)
        {
            modelBuilder.Entity<ProgramEntity>()
                .HasMany(p => p.Nodes)
                .WithOne(n => n.Program)
                .HasForeignKey(n => n.ProgramId)
                .OnDelete(DeleteBehavior.Cascade);
        }
    }

    public class ProgramEntity
    {
        public int Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public DateTime CreatedAt { get; set; }
        public DateTime LastModified { get; set; }
        
        // Navigation property
        public List<NodeEntity> Nodes { get; set; } = new List<NodeEntity>();
    }

    public class NodeEntity
    {
        public int Id { get; set; }
        public int ProgramId { get; set; }
        public ProgramEntity Program { get; set; } = null!; // Required reference
        
        public int ParentNodeId { get; set; } // For hierarchical worktree (0 = root)
        public int SequenceOrder { get; set; }
        
        public string NodeType { get; set; } = string.Empty; // e.g., "MoveNode", "WaitNode", "IO"
        
        // Serialized property JSON for flexible node parameters (Pose XYZW, Speed, Delay)
        public string ParametersJson { get; set; } = string.Empty;
    }

    public class RobotConfigEntity
    {
        public int Id { get; set; }
        public string ConfigKey { get; set; } = string.Empty;
        public string ConfigValueJson { get; set; } = string.Empty;
    }
}
