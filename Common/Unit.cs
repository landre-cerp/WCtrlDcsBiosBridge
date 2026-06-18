namespace WCtrlDcsBiosBridge.Common;

/// <summary>
/// Represents a void result type for operations that don't return a value.
/// </summary>
public readonly record struct Unit
{
    public static Unit Value { get; } = new();
}
