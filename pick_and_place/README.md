# Pick and Place demo

A demonstration of the Gaia Platform database working with declarative rules to run a naive pick and place application.

## Pre-requisites

Ubuntu 20.04 is required.

Follow the [getting started instructions](https://gaia-platform.github.io/gaia-platform-docs.io/articles/getting-started-with-gaia.html) to install the Gaia SDK.

## Build the Gaia application

To build a Gaia application you need the `gaia_db_server` running. If during the SDK installation you selected to install 
the `gaia_db_server` as a `systemd` service you won't need to do anything. Otherwise, you need to manually start the server
into a separate terminal:

```shell
# Start the server with persistence disabled
gaia_db_server --persistence disabled

# OR start the server with persistence enabled
gaia_db_server --data-dir .pick_and_place_db
```

Now you can build the application:

```shell
mkdir build
cd build/
cmake ..
make
```

This is what happens when building a Gaia application. Most of these steps are handled by the build system:

1. The schema (`gaia/pick_and_place.ddl`) is loaded into the `gaia_db_server` with `gaiac` tool.
2. `gaiac` also generates the [Direct Access Classes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/apps-direct-access.html)
    from the schema definition in the Database. Direct Access Classes allow you to perform Create/Read/Update/Delete 
    copy-free way, transactional, thread-safe way.
3. The ruleset (`gaia/pick_and_place.ruleset`) is translated into C++ code.
4. Your application (`src/main.cpp`) is built including the Direct Access Classes and the Translated Ruleset Code.

## Running the Demo

Now you can run:

```shell
./pick_and_place
```
