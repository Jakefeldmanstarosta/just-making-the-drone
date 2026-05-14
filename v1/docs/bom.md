# V1 Bill of Materials

Canonical parts list for the V1 drone. **Update this any time a part is added, swapped, or removed** — keep it in lockstep with the KiCad schematic.

Last updated: _fill in date_
Schematic revision: _fill in revision_

## Flight Controller PCB (Drone_FC_V1)

| Ref     | Part            | Manufacturer | Mfg Part #     | Supplier | Supplier Part # | Qty | Unit cost | Notes                       |
| ------- | --------------- | ------------ | -------------- | -------- | --------------- | --- | --------- | --------------------------- |
| U1      | ESP32 module    | Espressif    | _fill in_      | _fill_   | _fill_          | 1   | _$_       | MCU / wireless              |
| U2      | IMU             | InvenSense   | MPU-6000       | _fill_   | _fill_          | 1   | _$_       | 6-axis accel + gyro         |
| U3      | Buck regulator  | TI           | TPS5450        | _fill_   | _fill_          | 1   | _$_       | Main 5V rail                |
| U4      | Buck regulator  | _various_    | XL4015         | _fill_   | _fill_          | 1   | _$_       | Secondary rail              |
|         | ...             |              |                |          |                 |     |           | _add remaining parts_       |

## Drone airframe / mechanical

| Item            | Qty | Supplier | Part #      | Unit cost | Notes               |
| --------------- | --- | -------- | ----------- | --------- | ------------------- |
| Frame           | 1   | _fill_   | _fill_      | _$_       |                     |
| Motors          | 4   | _fill_   | _fill_      | _$_       |                     |
| ESCs            | 4   | _fill_   | _fill_      | _$_       |                     |
| Props           | 4+  | _fill_   | _fill_      | _$_       | Order spares        |
| Battery (LiPo)  | _n_ | _fill_   | _fill_      | _$_       |                     |

## Tooling / consumables (not shipped with product)

| Item       | Qty | Used for           |
| ---------- | --- | ------------------ |
| Solder     |     |                    |
| Heat shrink|     |                    |

## Totals

- **BOM cost per drone**: _calculate_
- **Tooling / one-time**: _calculate_

## Change log

<!-- Append a line every time the BOM changes. Newest at top. -->

- _YYYY-MM-DD_ — Initial template
