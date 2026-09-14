# Sandbox

The sandbox is the interactive test application for `zonai`.

It will provide:

- a Win32 application shell
- a minimal Direct3D 11 debug renderer
- Dear ImGui controls and diagnostics
- visual test scenes for physics features

The sandbox may depend on `zonai`, but the `zonai` physics library must never depend on the sandbox, Win32, Direct3D, or ImGui.
