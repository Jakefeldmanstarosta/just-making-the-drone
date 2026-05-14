# Just Making The Drone — Company Workspace

Welcome to the Just Making The Drone company repository. 

## Repository layout

The repo is organized **by product version** for engineering work, with company-wide business folders alongside.

```
.
├── v1/                  ← Drone V1 — hardware, firmware, testing, docs, logbooks
├── v2/                  ← Drone V2 — placeholder for next iteration
├── operations/          ← Internal ops: processes, vendors, facilities
├── marketing/           ← Brand, website, content, launch plans
├── finance/             ← Budgets, forecasts, invoices, expense tracking
├── legal/               ← Contracts, IP, compliance, NDAs
├── sales/               ← Pipeline, customers, pricing, decks
├── hr/                  ← Hiring, onboarding, policies, org chart
├── templates/           ← Reusable templates (logbook entries, etc.)
├── .github/             ← Issue and PR templates
├── CONTRIBUTING.md      ← How to work in this repo
└── README.md            ← You are here
```

## Working in a product version folder

Every product version (e.g. `v1/`) has the same internal layout so anyone can navigate any version:

```
v1/
├── README.md            ← Version overview, status, key links
├── hardware/            ← KiCad schematics, PCB, footprints
├── firmware/            ← Embedded code (.ino, libraries)
├── testing/             ← Bench tests, motor tests, flight tests
├── docs/                ← BOM, suppliers, specs, datasheets index
└── logbooks/            ← Dated engineering logbook entries
```

## Quick links

- [Contributing guide](./CONTRIBUTING.md)
- [V1 product README](./v1/README.md)
- [Logbook templates](./templates/)
- [V1 Bill of Materials](./v1/docs/bom.md)
- [V1 Suppliers](./v1/docs/suppliers.md)

## For new team members

1. Read this README and `CONTRIBUTING.md`.
2. Visit the product folder for the version you'll be working on (start with `v1/README.md`).
3. Check the latest logbook entries to see what the team has been doing.
4. Find your department folder (engineering = product version; otherwise `operations/`, `marketing/`, etc.) and read its README.
