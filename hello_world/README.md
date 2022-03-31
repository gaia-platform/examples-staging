# Writing a Gaia Application

Writing a Gaia application involves defining a schema and writing rules to act on your application's data. The schema 
and the rules are processed by gaia tools (`gaiac` and `gaiat`) to generate C++ code to be used in your application. 
The Sandbox takes care of these steps for you. 

Please refer to 
[Writing your first Gaia application](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tutorials/writing-first-gaia-application.html)
to build Gaia applications locally.

## Define the schema

Gaia provides a SQL-like Data Definition Language (DDL) to define your application's data model. The DDL is processed by 
[`gaiac`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiac.html) which loads the table
definitions into the database and generates the C++ code 
([Direct Access Classes](https://gaia-platform.github.io/gaia-platform-docs.io/articles/apps-direct-access.html)) 
to Create/Read/Update/Delete (CRUD) the data in a copy-free, 
transactional and thread-safe manner.

The following snippet defines the database `hello_world` and creates the table `person` in it. 
```sql
database hello_world

table person
(
    name string
)
```

`gaiac` process this definition and generates the class `gaia::hello_world::person_t` to perform CRUD operations on the
table `person` from your C++ code. Same thing happens with all the tables defined in the DDL.

## Define the rules

Gaia rules encode your application's logic. Rules are written in Gaia Declarative C++ which is a superset of C++ that,
being aware of the Gaia Schema, makes it straight forward to interact with data. The ruleset, the file that contains 
the rules, is processed by [`gaiat`](https://gaia-platform.github.io/gaia-platform-docs.io/articles/tools/tool-gaiat.html)
which translates them into C++ code.

The following snippet defines a ruleset with one rule. The rule `on_insert(person)` is invoked every time a record is
inserted in the table `person` (defined in the DDL). `person.name` access the `name` column in the current `person` record.
```cpp
ruleset hello_ruleset
{
    on_insert(person)
    {
        gaia_log::app().info("{} inserted!", person.name);
    }
```

The Gaia Rules Engine takes care of the transactions, scheduling of the rules, and rescheduling of the rules in case of 
error.

## The Main application

The lifecycle of a Gaia application starts from the C++ code. Rules react to database changes therefore you need to 
mutate the state of the database from C++ to trigger the first rule.

The following snippet initializes the Gaia system, and inserts a record into the `person` table. As soon as the 
transaction is committed (`gaia::db::commit_transaction()`), the rule engine will enqueue and execute the 
`on_insert(person)` rule, passing the `"Alice"` record asynchronously.

`gaia::system::shutdown()` waits for all the rules to be executed and shuts down the gaia system letting the execution 
complete.
```cpp
int main() {

  gaia::system::initialize();

  gaia::db::begin_transaction();
  gaia::hello_world::person_t::insert_row("Alice");
  gaia::db::commit_transaction();

  gaia::system::shutdown();
}
```

# Now it's your turn!

You can build the Hello World example by pressing the `Build` button. Once the build is completed you cna press run to 
see it running.

You can try to:
1. Uncomment `Rule 2` in the ruleset file.
2. Adding more `person` records in the `CPP` file.
