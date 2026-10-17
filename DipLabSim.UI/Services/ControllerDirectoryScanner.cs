using System;
using System.Collections.Generic;
using System.IO;
using System.Linq;
using DipLabSim.UI.Models;

namespace DipLabSim.UI.Services;

public sealed class ControllerDirectoryScanner : IDisposable
{
    private readonly string _controllersDirectory;
    private readonly FileSystemWatcher? _watcher;

    public event Action? ControllersChanged;

    public string ControllersDirectory => _controllersDirectory;

    public ControllerDirectoryScanner(string? targetDirectory = null)
    {
        _controllersDirectory = targetDirectory ?? Path.Combine(AppDomain.CurrentDomain.BaseDirectory, "Controllers");

        try
        {
            if (!Directory.Exists(_controllersDirectory))
            {
                Directory.CreateDirectory(_controllersDirectory);
            }

            _watcher = new FileSystemWatcher(_controllersDirectory, "*.dll")
            {
                NotifyFilter = NotifyFilters.FileName | NotifyFilters.LastWrite | NotifyFilters.CreationTime,
                EnableRaisingEvents = true
            };

            _watcher.Created += OnDirectoryChanged;
            _watcher.Deleted += OnDirectoryChanged;
            _watcher.Renamed += OnDirectoryChanged;
        }
        catch (Exception ex)
        {
            System.Diagnostics.Debug.WriteLine($"Failed to initialize FileSystemWatcher: {ex.Message}");
        }
    }

    private void OnDirectoryChanged(object sender, FileSystemEventArgs e)
    {
        ControllersChanged?.Invoke();
    }

    public IReadOnlyList<ControllerOption> Scan()
    {
        var list = new List<ControllerOption>
        {
            ControllerOption.None,
            ControllerOption.ExternalPythonGrpc
        };

        if (Directory.Exists(_controllersDirectory))
        {
            try
            {
                var dllFiles = Directory.EnumerateFiles(_controllersDirectory, "*.dll", SearchOption.TopDirectoryOnly)
                    .OrderBy(f => Path.GetFileName(f), StringComparer.OrdinalIgnoreCase);

                foreach (var dll in dllFiles)
                {
                    var fileName = Path.GetFileName(dll);
                    list.Add(new ControllerOption(
                        DisplayName: $"[Plugin] {fileName}",
                        FullPath: dll,
                        IsBuiltIn: false,
                        Description: $"Compiled plugin DLL located at: {dll}"));
                }
            }
            catch (Exception ex)
            {
                System.Diagnostics.Debug.WriteLine($"Error scanning controllers directory: {ex.Message}");
            }
        }

        return list;
    }

    public void Dispose()
    {
        if (_watcher != null)
        {
            _watcher.Created -= OnDirectoryChanged;
            _watcher.Deleted -= OnDirectoryChanged;
            _watcher.Renamed -= OnDirectoryChanged;
            _watcher.Dispose();
        }
    }
}
