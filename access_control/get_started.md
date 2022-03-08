# Gaia Access Control sample 
The Gaia Sandbox provides a containerized simulation environment that allows you to experiment with samples written using the Gaia Platform.

This sample models a security access control to allow or deny people entering an office campus. It contains:

- A headquarters (HQ) building
- Rooms inside the HQ building
- People, of varying access levels:
    - John: an employee with access to all rooms
    - Jane: a visitor with limited room access
    - A stranger: they are not allowed inside the HQ building.
- Event schedules for people and rooms

## Overview of the Sandbox
The Sandbox UI consists of the following elements:

### User interface window
The right side of the Sandbox shows the controls for the Access Control simulation. Each person in the **People** column has several icons: **Badge**, **Face**, **Parking**, and **WiFi**. These icons are all scanning options: for example, selecting somebody's **Badge** icon simulates a "badge scan" into a building.

Every person has a building access dropdown menu that contains the buildings and rooms that they can potentially access. Use the dropdown menu to choose a building or room, then select one of the scan icons to gain access to that building or room. Every building access dropdown menu starts with the **HQ Building** and does not expose the available rooms until a scan is performed and access granted. After access is granted to the building, the person can scan and attempt to access the rooms available in that building.

The **Location at time** control allows you to set the time of day, which is important for granting access to visitors such as Jane who cannot enter a room until a certain time. The fast-forward icon advances the time in 30-minute intervals.

### Ruleset, DDL, and Output tabs

There are three tabs on the left. These are windows that show:

- **Ruleset**: Displays the Declarative C++ code that handles all the "business logic" of building and room access. You can edit the code in this tab and rebuild the sample to see how it affects the way that the sample runs.
- **DDL**: Displays Data Definition Language (DDL) statements that describe the database schema of the "data model" for the Access Control scenario.
- **Output**: Displays informational details when you are building or running the sample.

## Build and run the sample
For the best user experience, keep the Sandbox's browser maximized to full-screen mode and use 1080p or higher resolution.

The Sandbox contains a complete sample for you to examine and modify.

While the sample builds, the **Output** window displays the progress of the build.

**Note**: It can take several minutes to build the sample.

If you press **Build** or **Run**, you will see progress messages in the **Output** tab.

When the sample has completed building, press **Run**.

### Example interaction
1. Build and run the sample.
2. Select John's **Badge** icon to scan his badge at the HQ building.
3. Select John's **Face** icon to scan his face at the HQ building and grant him access to the building.
4. Select Jane's **Badge** then **Face** icon. She will not be allowed in.
5. Select the **Fast-forward** button until it reads `10:00 am`.
6. Select Jane's **Badge** then **Face** icon to grant her access to the HQ building.
7. From John and Jane's building access dropdown menus, select **Big Room** instead of **Little Room**.
8. Select John and Jane's **Face** icons to scan them into the Big Room for their "boring meeting".

Try other combinations to see how the rules interact.

Now you can start experimenting with the sample by modifying the rules and DDL. After you edit the rules or DDL and rebuild the sample, run it and see how your changes affect building/room access.

For more information about Rulesets and Gaia DDL, see the [Gaia technical documentation](http://docs.gaiaplatform.io). 