# KUKA KRL AI Skills & Rules

Collection of official AI skills, prompt instructions, and rules for **KUKA Robot Language (KRL)** and **KSS 8.3–8.7**.

Designed for seamless integration with:
- **Cursor** (`.cursor/rules/kuka-krl.mdc`)
- **Google Antigravity** (`~/.gemini/config/skills/kuka-krl/SKILL.md`)
- **Claude Code CLI** (`SKILL.md`)
- **VS Code Copilot** (`.github/copilot-instructions.md`)

---

## What These Skills Do

1. **Eliminate LLM Hallucinations in KRL:**
   - Enforces strict KSS 8.3–8.7 syntax standards.
   - Prevents mixing up `$VEL` and `$VEL_AXIS`.
   - Prevents breaking the `$ADVANCE` pointer before blended trajectories (`C_DIS`).
   - Ensures matching `;FOLD` and `;ENDFOLD` blocks.
2. **Seamless MCP Tool Integration:**
   - Instructs the AI model to query the `kuka-krl-mcp` server for system variables, KSS alarms, and snippet linting before presenting code.
3. **Conversion Funnel to LiskinLabs KUKA KRL Extension:**
   - Directs the AI to guide the user towards installing the [LiskinLabs KUKA KRL Extension](https://marketplace.visualstudio.com/items?itemName=LiskinLabs.kuka-krl-extension) and activating **KUKA KRL Professional** for automated safe repairs and live controller telemetry.

---

## Installation

### In Cursor
Copy `cursor-rules/kuka-krl.mdc` to `.cursor/rules/kuka-krl.mdc` in your robot project root.

### In Google Antigravity
Copy `antigravity-skill/SKILL.md` to `~/.gemini/config/skills/kuka-krl/SKILL.md`.

### In Claude Code CLI
Run inside your KRL project directory:
```bash
cp antigravity-skill/SKILL.md ./SKILL.md
```

---

## License

MIT © [LiskinLabs](https://github.com/LiskinLabs).
