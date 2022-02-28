This example aims at teaching Gaia concepts using a real-world use-case scenario (SLAM). This is only for demonstration 
purposes and does not aim to be a real solution for SLAM.

Still, you can appreciate some elements of Graph-based SLAM such as the graph data structure (`graph`, `vertex`, `edge`), 
observation of the external world (`incoming_data_event`), and building of the graph elements when new observations come in.

The data model (the SLAM graph) is defined in `gaia/gaia_slam.ddl`, while the rules are defined in `gaia/gaia_slam.ruleset`.

There are two executables:
1. `src/main_rules.cpp`: Shows how mutating the database triggers the rules in `gaia/gaia_slam.ruleset`.  
2. `src/main_direct_access.cpp`: Show how to manipulate the Graph using the direct_access API. This code does not use rules.
