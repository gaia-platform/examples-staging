----------------------------------------------------
-- Copyright (c) Gaia Platform LLC
--
-- Use of this source code is governed by the MIT
-- license that can be found in the LICENSE.txt file
-- or at https://opensource.org/licenses/MIT.
----------------------------------------------------

create database if not exists incubator;

use incubator;

create table if not exists incubator (
    name string,
    is_on bool,
    min_temp float,
    max_temp float,
    sensors references sensor[],
    actuators references actuator[]
)

create table if not exists sensor (
    name string,
    timestamp uint64,
    value float,
    incubator references incubator
)

create table if not exists actuator (
    name string,
    timestamp uint64,
    value float,
    incubator references incubator
)
