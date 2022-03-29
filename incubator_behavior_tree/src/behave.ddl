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
    parent references node,
    check_temperature references check_temperature_trigger,
    set_fan_speed references set_fan_speed_trigger,
    check_fan_speed references check_fan_speed_trigger,
    adjust_fan_speed references adjust_fan_speed_trigger
)

-- Tables to trigger actions asynchronously.
table check_temperature_trigger (
    parent_node references node
)
table set_fan_speed_trigger (
    parent_node references node
)
table check_fan_speed_trigger (
    parent_node references node
)
table adjust_fan_speed_trigger (
    parent_node references node
)
