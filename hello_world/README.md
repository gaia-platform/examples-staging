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

The following snippet defines the database `hello_world` and creates the table `person`. 
```sql
database hello_world

table person
(
    name string
)
```

The `gaiac` tool processes this definition and generates the class `gaia::hello_world::person_t` to perform data 
operations on the table `person` from within your C++ code.

## Define your Rules

Rules encode your application's logic. Rules are written in Gaia Declarative C++, a schema-aware superset of C++ that 
makes it straightforward to interact with data. The ruleset (the file that contains the rules) is processed
by [`gaiat`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiat.html), which translates the
rules into C++ code

The following snippet defines a ruleset with one rule. The rule `on_insert(person)` is invoked every time a record is
inserted into the table `person` (defined in the DDL). The reference to `person.name` accesses the `name` column of the 
newly inserted  `person` record:
```cpp
ruleset hello_ruleset
{
    on_insert(person)
    {
        gaia_log::app().info("{} inserted!", person.name);
    }
}
```

The Gaia Rules Engine takes care of scheduling the rules, running the rules in a transaction, and rescheduling them
in case of error.

## The Main application

The lifecycle of a Gaia application starts in your application's C++ code. Rules react to changes in your data, so you 
need to change the database from your C++ code to trigger the first rule.

The following snippet initializes the Gaia system (`gaia::system::initialize`) and inserts a record into the `person` 
table. As soon as the transaction is committed (`gaia::db::commit_transaction()`), the rules engine enqueues and executes 
the `on_insert(person)` rule asynchronously, passing it the `"Alice"` record. Note that because Gaia manages your data 
in a transactional manner, all data must be accessed and updated from within a transaction.

When your application is done, call `gaia::system::shutdown()`. On shutdown, the rules engine will wait for all enqueued 
rules to be executed before exiting your program.
```cpp
int main()
{
    gaia::system::initialize();
    
    gaia::db::begin_transaction();
    gaia::hello_world::person_t::insert_row("Alice");
    gaia::db::commit_transaction();

    gaia::system::shutdown();
}
```

# Now it's your turn!

You can build the Hello World example by pressing the `Build` button. Once the build completes, press `Run` to 
run the example. After seeing the results, you can try to modify the code yourself. Here are a 
couple of suggestions:

1. Uncomment `Rule 2` in the ruleset file.
2. Add more `person` records in the `CPP` file.

# Next Steps
- [Direct Access Tutorial](https://sandbox.gaiaplatform.io/?scenario=direct_access): Interact with the Gaia
  Database using the Direct Access API.
- [Rules Tutorial](https://sandbox.gaiaplatform.io/?scenario=rules): Implement your business logic using the
  Gaia Declarative Language.
