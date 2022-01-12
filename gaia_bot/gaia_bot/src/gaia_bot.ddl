database gaia_bot

table ego (
    id uint8 unique,

    range_sensors
        references range_sensors[]
)

table range_sensors (
    id uint8,
    name string,
    distance uint16,

    ego
        references ego
            where range_sensors.ego_id = ego.id,
    ego_id uint8
);
