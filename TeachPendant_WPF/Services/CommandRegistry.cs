using System;
using System.Collections.Generic;
using System.Linq;
using System.Windows.Input;

namespace TeachPendant_WPF.Services
{
    /// <summary>
    /// A single command registration in the system.
    /// Used by toolbar, keybindings, command palette, and context menus.
    /// </summary>
    public class CommandDefinition
    {
        public string Id { get; set; } = string.Empty;
        public string DisplayName { get; set; } = string.Empty;
        public string Category { get; set; } = string.Empty;
        public string Description { get; set; } = string.Empty;
        public string IconGlyph { get; set; } = string.Empty;
        public KeyGesture? DefaultGesture { get; set; }
        public Func<bool>? CanExecute { get; set; }
        public Action? Execute { get; set; }
    }

    /// <summary>
    /// Central command registry — single source of truth for all app commands.
    /// Enables command palette, keybindings, context menus, and toolbar integration.
    /// </summary>
    public class CommandRegistry
    {
        private readonly Dictionary<string, CommandDefinition> _commands = new();

        public void Register(CommandDefinition command)
        {
            _commands[command.Id] = command;
        }

        public CommandDefinition? Get(string id)
            => _commands.TryGetValue(id, out var cmd) ? cmd : null;

        public IEnumerable<CommandDefinition> All()
            => _commands.Values;

        public IEnumerable<CommandDefinition> Search(string query)
        {
            if (string.IsNullOrWhiteSpace(query))
                return _commands.Values;

            return _commands.Values.Where(c =>
                c.DisplayName.Contains(query, StringComparison.OrdinalIgnoreCase) ||
                c.Category.Contains(query, StringComparison.OrdinalIgnoreCase) ||
                c.Id.Contains(query, StringComparison.OrdinalIgnoreCase));
        }

        public IEnumerable<CommandDefinition> ByCategory(string category)
            => _commands.Values.Where(c =>
                c.Category.Equals(category, StringComparison.OrdinalIgnoreCase));

        /// <summary>
        /// Create an ICommand adapter for use in XAML bindings.
        /// </summary>
        public ICommand CreateCommand(string id)
        {
            var def = Get(id);
            if (def == null) throw new ArgumentException($"Command '{id}' not registered");
            return new RegistryRelayCommand(def);
        }

        /// <summary>
        /// Get all registered KeyGestures for InputBindings.
        /// </summary>
        public IEnumerable<(KeyGesture gesture, CommandDefinition command)> GetKeyBindings()
        {
            foreach (var cmd in _commands.Values)
            {
                if (cmd.DefaultGesture != null)
                    yield return (cmd.DefaultGesture, cmd);
            }
        }
    }

    /// <summary>
    /// ICommand wrapper that delegates to a CommandDefinition from the registry.
    /// </summary>
    public class RegistryRelayCommand : ICommand
    {
        private readonly CommandDefinition _definition;

        public RegistryRelayCommand(CommandDefinition definition)
        {
            _definition = definition;
        }

        public bool CanExecute(object? parameter)
            => _definition.CanExecute?.Invoke() ?? true;

        public void Execute(object? parameter)
            => _definition.Execute?.Invoke();

        public event EventHandler? CanExecuteChanged
        {
            add => CommandManager.RequerySuggested += value;
            remove => CommandManager.RequerySuggested -= value;
        }
    }
}
