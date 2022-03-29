----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database behave

table node (
    -- A human readable name.
    name string,
    -- Status of a node: success, failure, running, idle.
    status uint8,
    -- Type of a node: selector, sequence, action.
    node_type uint8,

    tick_flag uint64,

    argument uint8,

    order uint8,

    children references node[],
    parent references node
);

-- Tables to trigger actions asynchronously.
table check_temperature_trigger (
);
table set_fan_speed_trigger (
);
table check_fan_speed_trigger (
);
table adjust_fan_speed_trigger (
);

create relationship if not exists check_temperature_trigger (
    node.check_temperature -> check_temperature_trigger,
    check_temperature_trigger.parent_node -> node
 );

create relationship if not exists set_fan_speed_trigger (
    node.set_fan_speed -> set_fan_speed_trigger,
    set_fan_speed_trigger.parent_node -> node
);

create relationship if not exists check_fan_speed_trigger (
    node.check_fan_speed -> check_fan_speed_trigger,
    check_fan_speed_trigger.parent_node -> node
);

create relationship if not exists adjust_fan_speed_trigger (
    node.adjust_fan_speed -> adjust_fan_speed_trigger,
    adjust_fan_speed_trigger.parent_node -> node
);
