using WwDevicesDotNet;

namespace WCtrlDcsBiosBridge.Aircrafts;

/// <summary>
/// CDU scratchpad buffer for Perf input pages (e.g. the A-10C takeoff page).
/// Collects typed characters from the CDU keyboard and exposes helpers for
/// committing values to input fields. Page routing is the caller's concern:
/// only call <see cref="HandleKey"/> while the input page is active.
/// </summary>
internal sealed class CduScratchpad
{
    public const int MaxLength = Metrics.Columns;

    public string Buffer { get; private set; } = "";
    public bool IsDeleteMode { get; private set; }

    /// <summary>Display string to render on the scratchpad line.</summary>
    public string DisplayText => IsDeleteMode ? "CLR" : Buffer;

    public bool HasContent => IsDeleteMode || Buffer.Length > 0;

    /// <summary>Raised whenever the buffer or delete-mode state changes.</summary>
    public event EventHandler? Changed;

    /// <summary>
    /// Feed a CDU key into the scratchpad.
    /// Returns <c>true</c> when the key was consumed (typing/erase keys).
    /// Returns <c>false</c> for LSKs, EXEC, and navigation keys — those are
    /// for the caller to handle.
    /// </summary>
    public bool HandleKey(Key key)
    {
        if (key == Key.Clr)
        {
            if (IsDeleteMode) { IsDeleteMode = false; Notify(); return true; }
            if (Buffer.Length > 0) { Buffer = Buffer[..^1]; Notify(); return true; }
            // Empty buffer: enter clear mode so the next LSK press clears the target field
            IsDeleteMode = true; Notify(); return true;
        }

        if (key == Key.Del)
        {
            if (Buffer.Length == 0) { IsDeleteMode = !IsDeleteMode; Notify(); return true; }
            return false;
        }

        if (key == Key.PositiveNegative)
        {
            AppendChar('-');
            return true;
        }

        var ch = key.ToCharacter(); // digits, A-Z, '.', '/', ' '
        if (ch.Length > 0)
        {
            AppendChar(ch[0]);
            return true;
        }

        return false;
    }

    /// <summary>
    /// Apply the scratchpad to a nullable field string and clear self.
    /// <list type="bullet">
    ///   <item>DELETE mode → sets field to <c>null</c> (clears it).</item>
    ///   <item>Buffer non-empty → sets field to the typed text.</item>
    ///   <item>Empty → no-op, returns unchanged field value.</item>
    /// </list>
    /// </summary>
    public string? CommitToField(ref string? currentValue)
    {
        if (IsDeleteMode) { currentValue = null; Clear(); return null; }
        if (Buffer.Length > 0) { currentValue = Buffer; Clear(); return currentValue; }
        return currentValue;
    }

    public void Clear()
    {
        Buffer = "";
        IsDeleteMode = false;
        Notify();
    }

    private void AppendChar(char c)
    {
        if (Buffer.Length >= MaxLength) return;
        if (IsDeleteMode) IsDeleteMode = false;
        Buffer += c;
        Notify();
    }

    private void Notify() => Changed?.Invoke(this, EventArgs.Empty);
}
