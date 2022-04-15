# Writing a Gaia Application

Writing a Gaia application involves defining a schema and writing rules that react to your application's data changes.
The schema and the rules are processed by Gaia tools (`gaiac` and `gaiat` respectively) to generate the C++ code used in
your application. The Sandbox takes care of these build steps for you.

Please refer to
[Writing your first Gaia application](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tutorials/writing-first-gaia-application.html),
to build Gaia applications locally.

## Define your Database Schema

Gaia provides a SQL-like Data Definition Language (DDL) to define your application's data model. The DDL is processed by
[`gaiac`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiac.html), which loads the table
definitions into the database and generates the C++ code
([Direct Access Classes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/apps-direct-access.html))
to enable copy-free reads as well as update, insert, and delete functionality to your data objects in a transactional,
and thread-safe manner.

The following snippet defines the database `hello_world` and creates the table `entity`.
```sql
database hello_world

table entity
(
    name string
)
```

The `gaiac` tool processes this definition and generates the class `gaia::hello_world::entity_t` to perform data
operations on the table `entity` from within your C++ code.

## Define your Rules

Rules encode your application's logic. Rules are written in Gaia Declarative C++, a schema-aware superset of C++ that
makes it straightforward to interact with data. The ruleset (the file that contains the rules) is processed
by [`gaiat`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiat.html), which translates the
rules into C++ code

The following snippet defines a ruleset with one rule. The rule `on_insert(entity)` is invoked every time a record is
inserted into the table `entity` (defined in the DDL). The reference to `entity.name` accesses the `name` column of the
newly inserted  `entity` record:
```cpp
ruleset hello_world_ruleset
{
    on_insert(entity)
    {
        gaia_log::app().info("{} inserted!", entity.name);
    }
}
```

The Gaia Rules Engine takes care of scheduling the rules, running the rules in a transaction, and retrying them should
they encounter an error.

## The Main application

The lifecycle of a Gaia application starts in your application's C++ code. Rules react to changes in your data, so you
need to change the database from your C++ code to trigger the first rule.

The following snippet initializes the Gaia system (`gaia::system::initialize`) and inserts a record into the `entity`
table. As soon as the transaction is committed (`gaia::db::commit_transaction()`), the rules engine enqueues and executes
the `on_insert(entity)` rule asynchronously, passing it the `"World"` record. Note that because Gaia manages your data
in a transactional manner, all data must be accessed and updated from within a transaction.

When your application is done, call `gaia::system::shutdown()`. On shutdown, the rules engine will wait for all enqueued
rules to be executed before exiting your program.
```cpp
int main()
{
    gaia::system::initialize();

    gaia::db::begin_transaction();
    gaia::hello_world::entity_t::insert_row("World");
    gaia::db::commit_transaction();

    gaia::system::shutdown();
}
```

# Now it's your turn!

You can build the Hello World example by pressing the `Build` button. Once the build completes, press `Run` to
run the example. After seeing the results, you can try to modify the code yourself. Here are a
couple of suggestions:

1. Uncomment `Rule 2` in the ruleset file.
2. Add more `entity` records in the `CPP` file.

# Next Steps
- <a href="https://sandbox.gaiaplatform.io/?scenario=direct_access" target="_parent">Direct Access Tutorial</a>: Interact with the Gaia
  Database using the Direct Access API.
- <a href="https://sandbox.gaiaplatform.io/?scenario=rules" target="_parent">Rules Tutorial</a>: Implement your business logic using the
  Gaia Declarative Language.
