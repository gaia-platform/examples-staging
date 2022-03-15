# Gaia Frequent Flyer Example

This example models a frequent flyer program.

The schema tracks the following the information:

- `airport`: Airport information.
- `airplane`: Information about the airplane.
- `route`: Information about route.
- `flight`: Information about the flight. A flight is connected in a:
  - 1:N relationship to travelers and segments.
  - 1:1 relationship with departure and arrival airports.
- `segment`: Describes the trip segment. A segments is connected in a:
  - 1:1 relationship to trips
  - 1:1 relationship to flights
- `trip`: Describes information about the itinerary for a single traveler. A trip is connected in a:
  - 1:1 relationship to airport for arriving flights
  - 1:1 relationship to airport for departing flights
- `traveler`: Information about the traveler. Travelers are connected to trip

The initial example defines three rules that use the traveler and flight tables

After you understand these rules you can extend the sample here in the sandbox.

## Rule 1

Tracks lifetime miles for the traveler.

- `flight_miles` from the `flights` table is defined as an [Active Field](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html?q=active%20fields).
- When the Frequent Flyer app updates the field in the database, the rules engine fires the rule.
- `lifetime_miles` indicates an implicit path `flight`->`segment`->`trip`->`traveler`.
- The Declarative C++ statement loops over each of the travelers on the flight and updates `lifetime_miles` for each.

## Rule 2

Evaluates and updates the member level of a traveler.

- `lifetime_miles` from the `traveler` table is defined as an [Active Field](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html?q=active%20fields).
- After Rule 1 updates `lifetime_miles`, the second rule fires for each traveler on the flight and updates their member status level, if necessary.

## Rule 3

Reacts to updates in the flight status.

- Uses the [on_update](https://gaia-platform.github.io/gaia-platform-docs.io/articles/reference/declarative-on_update.html) rule prefix to watch for updates to the `flight_status` field in the `trip` table.
- When the frequent flyer app updates flight status, the rules engine fires this rule which updates the trip segments when the flight status is set to landed.
- For more information on the on_ rule prefixes, see [on_xxxx Rule prefixes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html#on_xxxx-rule-prefixes) in the Gaia programming model documentation.

## Frequent Flyer app direct access code

The app uses the Direct Access classes to update the database.

- Explictly starts a transaction against the database. For more information, see [TBD]().
- Gets a list of flights that match the flight number and sets the cursor to the start of the list. With the sample data set, this cursor returns just one row or no rows.
- Iterate through the returned list of flights that have a status of "scheduled".
- Use the [Direct Access Classes API](https://gaia-platform.github.io/gaia-platform-docs.io/articles/reference/da-class-api.html) to instantiate a writer.
- Set the `flight_status` and `flight_miles` field values.
- Update the row.
- Commit the transaction.
