# Drone V1

The first generation of our drone. This folder contains everything needed to build, fly, and iterate on V1.

## Status

| Subsystem  | Status        | Owner | Notes                                   |
| ---------- | ------------- | ----- | --------------------------------------- |
| Hardware   | In development | TBD  | KiCad design in `hardware/Drone_FC_V1/` |
| Firmware   | In development | TBD  | Quadcopter flight controller (.ino)     |
| Testing    | In progress    | TBD  | Bench motor tests running               |

Update this table whenever a subsystem moves between *in development* / *integration* / *flight test* / *shipped*.

## Layout

- `hardware/Drone_FC_V1/` — KiCad project: schematic (`.kicad_sch`), PCB (`.kicad_pcb`), libraries, backups, datasheets.
- `firmware/quad_flight_controller.ino` — Main flight controller firmware (Arduino).
- `firmware/v1.ino` — Original v1 firmware sketch.
- `testing/4motorBluetooth.ino` — Four-motor Bluetooth bench test sketch.
- `docs/bom.md` — Bill of Materials (canonical parts list).
- `docs/suppliers.md` — Supplier directory (who we buy from, contact, lead time).
- `logbooks/` — Dated engineering logbook entries by month.

## Build instructions

<!-- Fill this in once we lock down the build process -->

1. Order the BOM from `docs/bom.md`.
2. Fab the PCB from `hardware/Drone_FC_V1/`.
3. Flash the firmware in `firmware/` using Arduino IDE.
4. Run bench tests from `testing/` before first flight.

## Key links

- [Bill of Materials](./docs/bom.md)
- [Supplier directory](./docs/suppliers.md)
- [Logbooks](./logbooks/)
- [Logbook entry template](../templates/logbook-daily-entry.md)
