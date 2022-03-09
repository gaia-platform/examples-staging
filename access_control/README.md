# [Gaia Platform](https://www.gaiaplatform.io/): Access Control Application
A demonstration of the Gaia Platform database working with declarative rules to run an access control system for an imaginary building. It communicates to a web GUI using MQTT and AWS IoT.

## Pre-requisites
Ubuntu 20.04 is required.

Follow the [getting started instructions](https://gaia-platform.github.io/gaia-platform-docs.io/articles/getting-started-with-gaia.html) to install the Gaia SDK.

## First-time setup
Open a terminal to the directory of this repo:
```
chmod u+x install_dependencies.sh rebuild.sh start_access_control.sh
./install_dependencies.sh
```

## Build and launch the Gaia application
Build the application with the script:
```
./rebuild.sh
```

Make sure the web sandbox is running, with the Access Control display showing "Unable to connect to Gaia!"

Export the remote client ID that you copied from the sandbox:
```
export REMOTE_CLIENT_ID=<something>
```

Run the application in the `build` directory:
```
cd build
. ../start_access_control.sh
```
The sandbox will now show the Access Control GUI.

## Experiment!
Now that everything is running the Gaia [rules](./src/access_control.ruleset) can be modified and extended to change behaviors. We encourage you to experiment to see how changes affect behavior and to imagine how Gaia could be used for other project ideas you may have.

If you changed the ruleset or the C++ code, rebuild it.
```
cd build
make -j$(nproc)
```

If you changed the schema DDL, run `./rebuild.sh`. This will erase the data in the database, but it's necessary to sync the schema with the state of the database.

If you have any questions or feedback please reach out to us at Gaia at [info@gaiaplatform.io](mailto:info@gaiaplatform.io).
