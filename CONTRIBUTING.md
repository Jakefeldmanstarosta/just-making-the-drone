# Contributing

This guide covers how we work in the Just Making The Drone repo — branching, commits, reviews, and where things go.

## Branching model

We use a simple trunk-based workflow:

- `main` — always shippable. Protected; changes land via pull request.
- `feat/<short-description>` — new features (e.g. `feat/v1-imu-calibration`).
- `fix/<short-description>` — bug fixes (e.g. `fix/v1-motor-stutter`).
- `hw/<short-description>` — hardware revisions (e.g. `hw/v1-power-rail-fix`).
- `docs/<short-description>` — documentation-only changes.

Open a draft PR early so the team can see what you're working on.

## Commit messages

Format: `<area>: <short summary>` followed by a blank line and a longer description if needed.

Areas: `v1-fw`, `v1-hw`, `v1-test`, `v2-*`, `ops`, `marketing`, `finance`, `legal`, `sales`, `hr`, `docs`, `meta`.

Examples:
- `v1-fw: add PID tuning for pitch axis`
- `v1-hw: swap TPS5450 for cheaper buck regulator`
- `ops: document weekly hardware standup cadence`

Keep commits focused. If a change touches firmware AND hardware AND docs, split it into separate commits.

## Pull requests

Every PR needs:

1. A clear title following the commit message format.
2. A filled-out PR description (template auto-loaded).
3. At least one reviewer from the relevant discipline.
4. A linked issue or logbook entry if the work isn't trivial.

## Where things go

| If you're adding...                         | Put it in...                                         |
| ------------------------------------------- | ---------------------------------------------------- |
| KiCad schematic / PCB for a version         | `v1/hardware/` (or `v2/hardware/`)                   |
| Flight controller code, motor code          | `v1/firmware/`                                       |
| Bench test rigs, flight test logs           | `v1/testing/`                                        |
| Bill of materials, supplier list, datasheet | `v1/docs/`                                           |
| Daily/weekly engineering notes              | `v1/logbooks/<Month-Year>/`                          |
| New supplier contract                       | `legal/contracts/`                                   |
| Marketing copy, social posts                | `marketing/`                                         |
| Budget, invoice, receipt                    | `finance/`                                           |
| Hiring docs, role descriptions              | `hr/`                                                |
| Internal process or SOP                     | `operations/`                                        |
| Reusable template (any kind)                | `templates/`                                         |

## Engineering logbook

We keep a dated engineering logbook in each version's `logbooks/` folder. The convention:

```
v1/logbooks/<Month-Year>/<YYYY-MM-DD>-<short-slug>.md
```

Use `templates/logbook-daily-entry.md` as your starting point. Add an entry at least once a week if you're touching the product — more often during heavy build sessions.

## Reviews

- **Firmware changes** — reviewer should compile & at minimum read through the diff for obvious issues. Flight-critical changes require a bench test note in the PR.
- **Hardware changes** — reviewer should open the KiCad project and run ERC/DRC. Document any change to the schematic in the PR description.
- **Business docs** — light review for clarity; the relevant department lead approves.

## Sensitive files

Do NOT commit:
- API keys, vendor passwords, customer PII
- Private financial data outside `finance/` (and even there, follow team policy)
- Large binaries unrelated to the product (use a shared drive for those)

If you're unsure whether something belongs in the repo, ask in #engineering before committing.
