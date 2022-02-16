# Gaia Frequent Flyer Example



This example models a frequent flyer program. The air

The schema tracks the following the information:

- airport: Airport information.
- airplane: Information about the airplane.
- route: Information about route.
- flight: Information about the flight. A flight is connected in a:
  - 1:n relationship to travelers and segments.  
  - 1:1 relationship with depart and arrival airports.
- segment: Describes the trip segment. A segments is connected in a:
  - 1:1 relationship to trips
  - 1:1 relationship to flights
- trip: Describes information about the itinerary for a single traveler. A trip is connected in a:
  - 1:1 relationship to airport for arriving flights
  - 1:1 relationship to airport for departing flights
- traveler: Information about the traveler. Travelers are connected to trip

The initial example defines three rules that use the traveler and flight tables

After you understand these rules you can extend the sample here in the sandbox.

## Rule 1

Tracks lifetime miles for the traveler.

- `flight_miles` from the flights table is defined as an [Active Field](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html?q=active%20fields).
- When the Frequent Flyer app updates the field in the database, the rules engine fires the rule.
- The lifetime_miles is an implicit path flight->segments->trip->traveler
- The Declarative C++ statement loops over each of the travelers on the flight and updates the lifetime_miles for each.

## Rule 2

Evaluates and updates the member level is for traveler. 

- `lifetime_miles` from the traveler table is defined as an [Active Field](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html?q=active%20fields).
- When Rule 1 updates `lifetime_miles`, this rule fires for each traveler on the flight which updates the member status level if necessary.

## Rule 3

Reacts to updates in the flight status.

- Uses the [on_update](https://gaia-platform.github.io/gaia-platform-docs.io/articles/reference/declarative-on_update.html) rule prefix to watch for updates to the flight_status field in the trip table.
- When the frequent flyer app updates flight status, the rules engine fires this rule which updates the trip segments when the flight status is set to landed.
- For more information on the on_ rule prefixes, see [on_xxxx Rule prefixes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/rulesets-gaia-programming-model.html#on_xxxx-rule-prefixes) in the Gaia programming model documentation.
