----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

database pick_and_place

-- Encapsulates the state of the machine.
-- Changes to the state triggers actions.
table state (
    pick bool,
    place bool
)

-- Denotes the action of picking.
table pick_action (
    done bool
)

-- Denotes the action of moving.
table move_action (
    done bool
)
