# Agent notes

Follow `.cursor/rules/` (always on):

1. Stay in this repo. No parent or system folders. Env vars (and registry only if required) are OK.
2. Commit each completed step with a why-focused message. Do not commit `build/` or `logs/`. Do not push unless asked.
3. Keep C++17 structure as-is. Test or bench every new feature (`tools/export_ekf_check.cpp`, `tools/check.bat`, or `drone_chase_sim.exe auto --quick`).
