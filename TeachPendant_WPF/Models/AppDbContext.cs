using System;
using System.Collections.Generic;
using Microsoft.EntityFrameworkCore;

namespace TeachPendant_WPF.Models
{
    public class AppDbContext : DbContext
    {
        // ── Core Tables ─────────────────────────────────────────────
        public DbSet<ProgramEntity> Programs { get; set; }
        public DbSet<NodeEntity> Nodes { get; set; }
        public DbSet<RobotConfigEntity> RobotConfigs { get; set; }

        // ── Phase C Expansion ───────────────────────────────────────
        public DbSet<WorldEntityRecord> WorldEntities { get; set; }
        public DbSet<WorkpieceRecord> Workpieces { get; set; }
        public DbSet<EncoderSettingsRecord> EncoderSettings { get; set; }
        public DbSet<ToolDefinitionRecord> ToolDefinitions { get; set; }
        public DbSet<ProgramVersionRecord> ProgramVersions { get; set; }

        public AppDbContext()
        {
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

            modelBuilder.Entity<ProgramEntity>()
                .HasMany(p => p.Versions)
                .WithOne(v => v.Program)
                .HasForeignKey(v => v.ProgramId)
                .OnDelete(DeleteBehavior.Cascade);
        }
    }

    // ── Existing Entities ────────────────────────────────────────────

    public class ProgramEntity
    {
        public int Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public DateTime CreatedAt { get; set; }
        public DateTime LastModified { get; set; }
        public List<NodeEntity> Nodes { get; set; } = new();
        public List<ProgramVersionRecord> Versions { get; set; } = new();
    }

    public class NodeEntity
    {
        public int Id { get; set; }
        public int ProgramId { get; set; }
        public ProgramEntity Program { get; set; } = null!;
        public int ParentNodeId { get; set; }
        public int SequenceOrder { get; set; }
        public string NodeType { get; set; } = string.Empty;
        public string ParametersJson { get; set; } = string.Empty;
    }

    public class RobotConfigEntity
    {
        public int Id { get; set; }
        public string ConfigKey { get; set; } = string.Empty;
        public string ConfigValueJson { get; set; } = string.Empty;
    }

    // ── NEW: Phase C Entities ────────────────────────────────────────

    public class WorldEntityRecord
    {
        public int Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public string EntityType { get; set; } = string.Empty; // Frame, Constraint, etc.
        public string TransformJson { get; set; } = string.Empty;
        public int? ParentId { get; set; }
        public string PropertiesJson { get; set; } = string.Empty;
    }

    public class WorkpieceRecord
    {
        public int Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public string FilePath { get; set; } = string.Empty; // STL/STEP file
        public string TransformJson { get; set; } = string.Empty;
        public bool IsVisible { get; set; } = true;
    }

    public class EncoderSettingsRecord
    {
        public int Id { get; set; }
        public int JointIndex { get; set; }
        public int Resolution { get; set; } = 4096;
        public double GearRatio { get; set; } = 100.0;
        public double OffsetDeg { get; set; }
        public bool IsInverted { get; set; }
    }

    public class ToolDefinitionRecord
    {
        public int Id { get; set; }
        public string Name { get; set; } = string.Empty;
        public double OffsetX { get; set; }
        public double OffsetY { get; set; }
        public double OffsetZ { get; set; }
        public double RotRx { get; set; }
        public double RotRy { get; set; }
        public double RotRz { get; set; }
        public double Mass { get; set; }
    }

    public class ProgramVersionRecord
    {
        public int Id { get; set; }
        public int ProgramId { get; set; }
        public ProgramEntity Program { get; set; } = null!;
        public int VersionNumber { get; set; }
        public DateTime CreatedAt { get; set; }
        public string SnapshotJson { get; set; } = string.Empty;
    }
}
