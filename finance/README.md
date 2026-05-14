# Finance

Budgets, forecasts, invoices, expense tracking, and anything money-related.

## What lives here

- `budgets/` — Annual and quarterly budgets, broken down by department.
- `forecasts/` — Revenue and runway projections.
- `invoices/` — Outgoing invoices to customers; incoming invoices from suppliers (or links to accounting system).
- `expenses/` — Expense reports, receipts.
- `taxes/` — Tax filings, sales tax, R&D credit documentation.
- `banking/` — Account info (no passwords!), wire instructions, reconciliations.

## Don't commit

- Passwords or full bank account numbers
- Employee PII (salaries, SSNs, etc.) — those go in `hr/` with appropriate access controls
- Customer payment details (PCI scope)

## Recurring tasks

| Cadence  | Task                          | Owner | Notes                                |
| -------- | ----------------------------- | ----- | ------------------------------------ |
| Weekly   | Reconcile bank transactions   | TBD   |                                      |
| Monthly  | Close the books               | TBD   | Update forecasts                     |
| Quarterly| Board / investor update       | TBD   | Pull from forecasts/ + product KPIs  |
| Annually | Tax filing                    | TBD   |                                      |
