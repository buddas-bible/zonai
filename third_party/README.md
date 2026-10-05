# Third Party

Dear ImGui 1.92.9b is vendored in `imgui/` for the Windows Sandbox/debug UI, with the Win32 and Direct3D 11 backends. The checked-in sources define the dependency used by this build; no download is needed during configuration.

The original [MIT license](imgui/LICENSE.txt) is retained. ImGui and platform headers are confined to the Sandbox and do not appear in the `zonai` physics API. Dependency upgrades should be deliberate changes with Sandbox build verification.
