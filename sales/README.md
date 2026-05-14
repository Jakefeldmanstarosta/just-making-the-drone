# Sales

Pipeline, customers, pricing, decks, and outbound.

## What lives here

- `pipeline/` — Active deals (or a link to CRM if we use one).
- `customers/` — Customer accounts, contacts, contract status.
- `pricing/` — Price lists, discount policies, volume tiers.
- `decks/` — Sales decks and one-pagers (build from `marketing/brand/`).
- `outreach/` — Email templates, prospect lists, campaign notes.

## Standard sales motion

1. Lead enters pipeline via inbound (website) or outbound (campaign).
2. Discovery call → qualification.
3. Demo / technical deep-dive.
4. Quote (use templates in `pricing/`).
5. Contract (use templates in `../legal/contracts/templates/`).
6. Close → handoff to operations for fulfillment.

## Don't commit

- Customer PII beyond contact info needed to run the relationship
- Negotiation strategy that would burn us if leaked
