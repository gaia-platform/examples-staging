# Writing a Gaia Application

Writing a Gaia application involves defining a schema and writing rules to act on your application's data. The schema 
and the rules are processed by Gaia tools, `gaiac` and `gaiat` respectively,  to generate C++ code to be used in your 
application. The Sandbox takes care of these steps for you. 

Please refer to 
[Writing your first Gaia application](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tutorials/writing-first-gaia-application.html)
to build Gaia applications locally.

## Define the schema

Gaia provides a SQL-like Data Definition Language (DDL) to define your application's data model. The DDL is processed by 
[`gaiac`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiac.html) which loads the table
definitions into the database and generates the C++ code 
([Direct Access Classes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/apps-direct-access.html)) 
to Create/Read/Update/Delete (CRUD) the data in a copy-free, 
transactional, and thread-safe manner.

The following snippet defines the database `hello_world` and creates the table `person` in it. 
```sql
database hello_world

table person
(
    name string
)
```

`gaiac` processes this definition and generates the class `gaia::hello_world::person_t` to perform CRUD operations on the
table `person` from your C++ code. The same thing happens with all the tables defined in the DDL.

## Define the rules

Gaia rules encode your application's logic. Rules are written in Gaia Declarative C++, which is a schema-aware superset 
of C++ that makes it straightforward to interact with data". The ruleset (the file that contains the rules) is processed
by [`gaiat`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiat.html), which translates the
rules into C++ code

The following snippet defines a ruleset with one rule. The rule `on_insert(person)` is invoked every time a record is
inserted into the table `person` (defined in the DDL). `person.name` accesses the `name` column in the current `person`
record.
```cpp
ruleset hello_ruleset
{
    on_insert(person)
    {
        gaia_log::app().info("{} inserted!", person.name);
    }
```

The Gaia Rules Engine takes care of scheduling the rules, running the rules into a transaction, and rescheduling them
in case of error.

## The main application

The lifecycle of a Gaia application starts in your application's C++ code. Rules react to database changes, so you need
to mutate the database from your C++ code to trigger the first rule.

The following snippet initializes the Gaia system and inserts a record into the `person` table. As soon as the 
transaction is committed (`gaia::db::commit_transaction()`), the rules engine enqueue and execute the 
`on_insert(person)` rule asynchronously, passing the `"Alice"` record.

`gaia::system::shutdown()` waits for all enqueued rules to be executed and then shuts down the Gaia system.
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

You can build the Hello World example by pressing the `Build` button. Once the build completes, you can press `Run` to 
run the example.

You can try to:
1. Uncomment `Rule 2` in the ruleset file.
2. Add more `person` records in the `CPP` file.
