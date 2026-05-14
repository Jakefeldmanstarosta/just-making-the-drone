# Legal

Contracts, intellectual property, compliance, and risk.

## What lives here

- `contracts/` — Customer agreements, supplier contracts, NDAs, partnerships.
  - `contracts/templates/` — Our standard contract templates.
  - `contracts/active/` — Fully executed contracts currently in force.
  - `contracts/archive/` — Expired or terminated contracts.
- `ip/` — Patents, trademarks, copyrights — applications and grants.
- `compliance/` — Regulatory compliance: FAA, FCC, CE, RoHS, etc.
- `policies/` — Public-facing terms of service, privacy policy.
- `incidents/` — Log of legal incidents and how they were resolved.

## Drone-specific compliance to track

| Regulator | What it covers                | Status          | Next action |
| --------- | ----------------------------- | --------------- | ----------- |
| FAA       | Drone operation / Part 107    | _evaluate_      |             |
| FCC       | Wireless (ESP32 emissions)    | _evaluate_      |             |
| CE        | EU sales                      | _not yet_       |             |
| RoHS      | Restricted substances         | _BOM review_    |             |

## Don't commit

- Anything covered by attorney-client privilege without lead's approval
- Counterparty's confidential information beyond what's in the executed contract
