using System.Collections.Generic;
using System.Linq;
using System.Threading.Tasks;
using Microsoft.EntityFrameworkCore;
using TeachPendant_WPF.Models;

namespace TeachPendant_WPF.Services
{
    public class DatabaseService
    {
        private readonly AppDbContext _context;

        public DatabaseService()
        {
            _context = new AppDbContext();
        }

        public async Task<List<ProgramEntity>> GetAllProgramsAsync()
        {
            return await _context.Programs.OrderByDescending(p => p.LastModified).ToListAsync();
        }

        public async Task<ProgramEntity?> GetProgramByIdAsync(int id)
        {
            return await _context.Programs
                .Include(p => p.Nodes)
                .FirstOrDefaultAsync(p => p.Id == id);
        }

        public async Task<ProgramEntity> CreateNewProgramAsync(string name)
        {
            var program = new ProgramEntity
            {
                Name = name,
                CreatedAt = System.DateTime.Now,
                LastModified = System.DateTime.Now
            };

            _context.Programs.Add(program);
            await _context.SaveChangesAsync();
            return program;
        }

        public async Task SaveProgramAsync(ProgramEntity program)
        {
            program.LastModified = System.DateTime.Now;
            _context.Programs.Update(program);
            await _context.SaveChangesAsync();
        }
        
        public async Task DeleteProgramAsync(int id)
        {
            var program = await _context.Programs.FindAsync(id);
            if (program != null)
            {
                _context.Programs.Remove(program);
                await _context.SaveChangesAsync();
            }
        }
    }
}
